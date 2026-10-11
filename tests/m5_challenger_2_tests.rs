//! Empirical Challenger 2 Adversarial Stress Test Suite for Milestone 5
//! End-to-End Pipeline, CAM Fabrication Output Integrity, DRC Stress, and Non-Regression
//!
//! Areas Covered:
//! 1. Full E2E Pipeline: Layout -> Dynamic Ratsnest -> 2-Layer Autorouter -> DRC -> KiCad/Gerber/Drill CAM Export
//! 2. Autorouting & DRC Stress on Reference Board (create_reference_board, 0 shorts, 0 clearance, 0 annular ring, 0 edge keepout, 0 unrouted nets)
//! 3. Adversarial DRC Violation Injections on Reference Board (shorts, clearance, annular ring, drill breakout, edge keepout, outside board boundary)
//! 4. KiCad S-Expression Full AST & Depth Oracle (Parenthesis depth strictly > 0 and depth = 0 at EOF, all footprints, nets, pads, tracks, vias, edge cuts)
//! 5. RS-274X Gerber 6-Layer Aperture & Geometric Syntax Oracle (D10+ apertures, standard formats, nanometer scaling, draws & flashes, M02*)
//! 6. Excellon NC Drill Tooling & Hit Distribution Oracle (M48, tool sorting, exact hit counts for THT headers and vias, deduplication of coincident vias, M30)
//! 7. Dynamic Ratsnest Invalidation & Copper-Connected Island Clustering
//! 8. Router Parameter Edge Cases & Boundary Rejections

use netlist_shatter_viz::pcb::*;
use std::collections::{BTreeMap, HashSet};

// ============================================================================
// Independent RS-274X, Excellon, and KiCad S-Expression Parsing Oracles
// ============================================================================

#[derive(Clone, Debug, PartialEq)]
enum ParsedAperture {
    Circle { dia: f64 },
    Rect { w: f64, h: f64 },
    Obround { w: f64, h: f64 },
}

#[allow(dead_code)]
#[derive(Clone, Debug)]
struct ParsedFlash {
    pub dcode: u32,
    pub x_nm: i64,
    pub y_nm: i64,
}

#[allow(dead_code)]
#[derive(Clone, Debug)]
struct ParsedDraw {
    pub dcode: u32,
    pub start_x_nm: i64,
    pub start_y_nm: i64,
    pub end_x_nm: i64,
    pub end_y_nm: i64,
}

#[derive(Clone, Debug)]
struct ParsedGerber {
    pub has_format: bool,
    pub has_momm: bool,
    pub has_lpd: bool,
    pub ends_with_m02: bool,
    pub apertures: BTreeMap<u32, ParsedAperture>,
    pub flashes: Vec<ParsedFlash>,
    pub draws: Vec<ParsedDraw>,
}

fn parse_gerber(content: &str) -> ParsedGerber {
    let mut has_format = false;
    let mut has_momm = false;
    let mut has_lpd = false;
    let mut ends_with_m02 = false;
    let mut apertures = BTreeMap::new();
    let mut flashes = Vec::new();
    let mut draws = Vec::new();

    let mut current_dcode: Option<u32> = None;
    let mut pending_move: Option<(i64, i64)> = None;

    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }

        if line == "%FSLAX46Y46*%" {
            has_format = true;
        } else if line == "%MOMM*%" {
            has_momm = true;
        } else if line == "%LPD*%" {
            has_lpd = true;
        } else if line == "M02*" {
            ends_with_m02 = true;
        } else if line.starts_with("%ADD") && line.ends_with("*%") {
            let inner = &line[4..line.len() - 2];
            if let Some(comma_pos) = inner.find(',') {
                let code_type = &inner[..comma_pos];
                let params = &inner[comma_pos + 1..];
                let type_char = code_type.chars().last().unwrap();
                let code_str = &code_type[..code_type.len() - 1];
                let code: u32 = code_str.parse().expect("valid dcode integer");

                match type_char {
                    'C' => {
                        let dia: f64 = params.parse().expect("circle diameter");
                        apertures.insert(code, ParsedAperture::Circle { dia });
                    }
                    'R' => {
                        let dims: Vec<&str> = params.split('X').collect();
                        assert_eq!(dims.len(), 2, "Rect dimensions format wXh");
                        let w: f64 = dims[0].parse().unwrap();
                        let h: f64 = dims[1].parse().unwrap();
                        apertures.insert(code, ParsedAperture::Rect { w, h });
                    }
                    'O' => {
                        let dims: Vec<&str> = params.split('X').collect();
                        assert_eq!(dims.len(), 2, "Obround dimensions format wXh");
                        let w: f64 = dims[0].parse().unwrap();
                        let h: f64 = dims[1].parse().unwrap();
                        apertures.insert(code, ParsedAperture::Obround { w, h });
                    }
                    other => panic!("Unsupported aperture type: {}", other),
                }
            }
        } else if line.starts_with('D') && line.ends_with('*') && !line.contains('X') && !line.contains('Y') {
            let code_str = &line[1..line.len() - 1];
            let code: u32 = code_str.parse().expect("dcode selection");
            current_dcode = Some(code);
        } else if line.contains("D03*") {
            let dcode = current_dcode.expect("Active aperture required for D03 flash");
            let (x, y) = parse_xy_nm(line);
            flashes.push(ParsedFlash { dcode, x_nm: x, y_nm: y });
        } else if line.contains("D02*") {
            let (x, y) = parse_xy_nm(line);
            pending_move = Some((x, y));
        } else if line.contains("D01*") {
            let dcode = current_dcode.expect("Active aperture required for D01 draw");
            let (end_x, end_y) = parse_xy_nm(line);
            let (start_x, start_y) = pending_move.expect("Pending D02 move required before D01 draw");
            draws.push(ParsedDraw {
                dcode,
                start_x_nm: start_x,
                start_y_nm: start_y,
                end_x_nm: end_x,
                end_y_nm: end_y,
            });
            pending_move = Some((end_x, end_y));
        }
    }

    ParsedGerber {
        has_format,
        has_momm,
        has_lpd,
        ends_with_m02,
        apertures,
        flashes,
        draws,
    }
}

fn parse_xy_nm(line: &str) -> (i64, i64) {
    let x_idx = line.find('X').expect("line contains X");
    let y_idx = line.find('Y').expect("line contains Y");
    let d_idx = line.find('D').expect("line contains D");

    let x_str = &line[x_idx + 1..y_idx];
    let y_str = &line[y_idx + 1..d_idx];

    let x: i64 = x_str.parse().expect("valid integer X coordinate in nanometers");
    let y: i64 = y_str.parse().expect("valid integer Y coordinate in nanometers");

    (x, y)
}

#[derive(Clone, Debug)]
struct ParsedDrill {
    pub starts_with_m48: bool,
    pub has_metric: bool,
    pub ends_with_m30: bool,
    pub tools: BTreeMap<u32, f64>,
    pub hits_by_tool: BTreeMap<u32, Vec<Point>>,
}

fn parse_drill(content: &str) -> ParsedDrill {
    let mut starts_with_m48 = false;
    let mut has_metric = false;
    let mut ends_with_m30 = false;
    let mut tools = BTreeMap::new();
    let mut hits_by_tool = BTreeMap::new();

    let mut current_tool: Option<u32> = None;
    let mut in_header = true;

    for (idx, line) in content.lines().enumerate() {
        let line = line.trim();
        if idx == 0 && line == "M48" {
            starts_with_m48 = true;
        }
        if line == "METRIC" {
            has_metric = true;
        }
        if line == "%" {
            in_header = false;
        }
        if line == "M30" {
            ends_with_m30 = true;
        }

        if in_header {
            if line.starts_with('T') && line.contains('C') {
                let c_idx = line.find('C').unwrap();
                let tool_num: u32 = line[1..c_idx].parse().expect("tool number");
                let dia: f64 = line[c_idx + 1..].parse().expect("tool diameter");
                tools.insert(tool_num, dia);
            }
        } else {
            if line.starts_with('T') && !line.contains('C') {
                let tool_num: u32 = line[1..].parse().expect("active tool number");
                current_tool = Some(tool_num);
            } else if line.starts_with('X') && line.contains('Y') {
                let tool_num = current_tool.expect("Tool must be selected before hits");
                let y_idx = line.find('Y').unwrap();
                let x: f64 = line[1..y_idx].parse().expect("drill hit X");
                let y: f64 = line[y_idx + 1..].parse().expect("drill hit Y");
                hits_by_tool.entry(tool_num).or_insert_with(Vec::new).push(Point::new(x, y));
            }
        }
    }

    ParsedDrill {
        starts_with_m48,
        has_metric,
        ends_with_m30,
        tools,
        hits_by_tool,
    }
}

// ----------------------------------------------------------------------------
// KiCad S-Expression Lexer & Depth Verifier
// ----------------------------------------------------------------------------

#[derive(Debug, PartialEq, Eq)]
enum SExprToken {
    OpenParen,
    CloseParen,
    Atom(String),
}

fn tokenize_sexpr(s: &str) -> Vec<SExprToken> {
    let mut tokens = Vec::new();
    let mut chars = s.chars().peekable();

    while let Some(&c) = chars.peek() {
        if c.is_whitespace() {
            chars.next();
        } else if c == '(' {
            tokens.push(SExprToken::OpenParen);
            chars.next();
        } else if c == ')' {
            tokens.push(SExprToken::CloseParen);
            chars.next();
        } else if c == '"' {
            chars.next(); // consume opening quote
            let mut string_val = String::new();
            while let Some(&ch) = chars.peek() {
                if ch == '"' {
                    chars.next(); // consume closing quote
                    break;
                } else if ch == '\\' {
                    chars.next();
                    if let Some(escaped) = chars.next() {
                        string_val.push(escaped);
                    }
                } else {
                    string_val.push(ch);
                    chars.next();
                }
            }
            tokens.push(SExprToken::Atom(string_val));
        } else {
            let mut atom = String::new();
            while let Some(&ch) = chars.peek() {
                if ch.is_whitespace() || ch == '(' || ch == ')' || ch == '"' {
                    break;
                }
                atom.push(ch);
                chars.next();
            }
            tokens.push(SExprToken::Atom(atom));
        }
    }
    tokens
}

fn verify_kicad_sexpr_depth_profile(content: &str) -> (usize, usize, usize) {
    let tokens = tokenize_sexpr(content);
    let mut current_depth: isize = 0;
    let mut max_depth: usize = 0;
    let mut open_count: usize = 0;
    let mut close_count: usize = 0;

    for (idx, token) in tokens.iter().enumerate() {
        match token {
            SExprToken::OpenParen => {
                current_depth += 1;
                open_count += 1;
                if current_depth as usize > max_depth {
                    max_depth = current_depth as usize;
                }
            }
            SExprToken::CloseParen => {
                current_depth -= 1;
                close_count += 1;
                if idx < tokens.len() - 1 {
                    assert!(
                        current_depth > 0,
                        "KiCad S-expression depth dropped to 0 prematurely at token {} / {}",
                        idx,
                        tokens.len()
                    );
                }
            }
            SExprToken::Atom(_) => {}
        }
    }

    assert_eq!(
        current_depth, 0,
        "Parenthesis depth at EOF must be EXACTLY 0, found {}",
        current_depth
    );
    assert_eq!(
        open_count, close_count,
        "Total open parens ({}) must equal close parens ({})",
        open_count, close_count
    );

    (open_count, close_count, max_depth)
}

// ============================================================================
// Task 1 & 2: Full End-to-End Pipeline & Reference Board Autoroute / DRC Stress
// ============================================================================

#[test]
fn test_reference_board_e2e_pipeline_and_drc_stress() {
    // 1. Create Reference Board
    let mut board = create_reference_board();

    // Verify Board Geometry & Footprint Inventory
    assert_eq!(board.width, 65.0);
    assert_eq!(board.height, 32.0);
    assert_eq!(board.footprints.len(), 11);

    let total_pads: usize = board.footprints.iter().map(|fp| fp.pads.len()).sum();
    assert_eq!(total_pads, 98);

    // Verify Initial Dynamic Ratsnest before routing
    let initial_ratsnest = compute_ratsnest(&board);
    assert_eq!(initial_ratsnest.len(), 31, "Reference board must have 31 unrouted EMST lines");

    // All initial ratsnest lines must connect valid pads with positive distance
    for line in &initial_ratsnest {
        assert!(line.start.distance_to(line.end) > 0.001);
        assert!(line.net > 0);
    }

    // 2. Configure Autorouter
    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    };

    // 3. Execute Autorouting
    let route_result = route_board(&mut board, &config);
    assert!(route_result.is_ok(), "Reference board autorouting must succeed: {:?}", route_result);

    // Verify generated tracks and vias count
    assert!(board.tracks.len() >= 90, "Autorouter must generate >= 90 track segments, found {}", board.tracks.len());
    assert!(board.vias.len() >= 30, "Autorouter must insert >= 30 vias, found {}", board.vias.len());

    // 4. Run Automated Geometric DRC
    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "Clean reference board must have ZERO short circuits: {:?}", drc.short_circuits);
    assert_eq!(drc.clearance_violations.len(), 0, "Clean reference board must have ZERO clearance violations: {:?}", drc.clearance_violations);
    assert_eq!(drc.annular_ring_violations.len(), 0, "Clean reference board must have ZERO annular ring violations: {:?}", drc.annular_ring_violations);
    assert_eq!(drc.edge_keepout_violations.len(), 0, "Clean reference board must have ZERO edge keepout violations: {:?}", drc.edge_keepout_violations);
    assert_eq!(drc.unrouted_nets.len(), 0, "Clean reference board must have ZERO unrouted nets: {:?}", drc.unrouted_nets);
    assert!(drc.is_clean(), "DRC report is_clean() must be true");

    // 5. CAM Fabrication Verification: KiCad Export
    let kicad_out = export_kicad_pcb(&board);
    let (open_p, _close_p, max_d) = verify_kicad_sexpr_depth_profile(&kicad_out);
    assert!(open_p >= 900, "KiCad S-expr must have >= 900 parens, found {}", open_p);
    assert!(max_d >= 3, "KiCad nesting depth must reach at least 3, found {}", max_d);

    // 6. CAM Fabrication Verification: RS-274X Gerber Exporter (All 6 Layers)
    let layers = [
        (Layer::FCu, "F_Cu"),
        (Layer::BCu, "B_Cu"),
        (Layer::FSilkscreen, "F_SilkS"),
        (Layer::EdgeCuts, "Edge_Cuts"),
        (Layer::FMask, "F_Mask"),
        (Layer::BMask, "B_Mask"),
    ];

    for (layer, name) in layers {
        let gbr_content = export_gerber_layer(&board, layer);
        let parsed = parse_gerber(&gbr_content);
        assert!(parsed.has_format, "Layer {} missing %FSLAX46Y46*%", name);
        assert!(parsed.has_momm, "Layer {} missing %MOMM*%", name);
        assert!(parsed.has_lpd, "Layer {} missing %LPD*%", name);
        assert!(parsed.ends_with_m02, "Layer {} must terminate with M02*", name);
        assert!(!parsed.apertures.is_empty(), "Layer {} must define apertures", name);
    }

    // 7. CAM Fabrication Verification: Excellon Drill Exporter
    let drl_content = export_excellon_drill(&board);
    let parsed_drill = parse_drill(&drl_content);
    assert!(parsed_drill.starts_with_m48, "Drill file must start with M48");
    assert!(parsed_drill.has_metric, "Drill file must specify METRIC");
    assert!(parsed_drill.ends_with_m30, "Drill file must end with M30");
    assert_eq!(parsed_drill.tools.len(), 2, "Drill file must define exactly 2 tools (0.40mm via and 1.00mm THT)");
}

// ============================================================================
// Task 2: CAM Fabrication Verification Deep Invariants
// ============================================================================

#[test]
fn test_kicad_s_expression_strict_ast_integrity() {
    let board = create_reference_board();
    let kicad_str = export_kicad_pcb(&board);

    // Verify depth profile
    let (open_cnt, close_cnt, _) = verify_kicad_sexpr_depth_profile(&kicad_str);
    assert_eq!(open_cnt, close_cnt);

    let tokens = tokenize_sexpr(&kicad_str);

    // Token 0: OpenParen, Token 1: Atom("kicad_pcb")
    assert_eq!(tokens[0], SExprToken::OpenParen);
    assert_eq!(tokens[1], SExprToken::Atom("kicad_pcb".to_string()));

    // Verify netlist table contains (net 0 "") and all board nets
    assert!(kicad_str.contains("(net 0 \"\")"), "Must contain mandatory KiCad default (net 0 \"\")");
    for (id, name) in &board.net_names {
        let expected = format!("(net {} \"{}\")", id, name);
        assert!(kicad_str.contains(&expected), "Must declare net definition: {}", expected);
    }

    // Verify all 11 component footprints exist with correct reference designators
    let expected_refs = ["U1", "U2", "J1", "Y1", "C1", "C2", "C3", "R1", "SW1", "J2", "J3"];
    for r in &expected_refs {
        let ref_pattern = format!("\"{}\"", r);
        assert!(kicad_str.contains(&ref_pattern), "KiCad output must declare component {}", r);
    }

    // Verify Edge.Cuts boundary lines
    assert!(kicad_str.contains("\"Edge.Cuts\""), "Must contain Edge.Cuts layer");
    assert!(kicad_str.contains("(gr_line"), "Must contain graphical line segments");
}

#[test]
fn test_gerber_rs274x_all_6_layers_full_aperture_and_coordinate_oracle() {
    let mut board = create_reference_board();
    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    };
    route_board(&mut board, &config).expect("Routing must succeed");

    // 1. F_Cu Layer: Must have track draws and pad flashes
    let fcu = export_gerber_layer(&board, Layer::FCu);
    let parsed_fcu = parse_gerber(&fcu);
    assert!(parsed_fcu.apertures.contains_key(&10), "F_Cu must have D10 aperture");
    assert!(!parsed_fcu.draws.is_empty(), "F_Cu must have track draws (D01)");
    assert!(!parsed_fcu.flashes.is_empty(), "F_Cu must have pad flashes (D03)");

    // Aperture D10 must be a circle matching router trace width 0.25 mm
    let d10 = parsed_fcu.apertures.get(&10).unwrap();
    assert_eq!(*d10, ParsedAperture::Circle { dia: 0.25 }, "D10 aperture must be 0.25mm trace");

    // 2. B_Cu Layer: Must have track draws and via/THT pad flashes
    let bcu = export_gerber_layer(&board, Layer::BCu);
    let parsed_bcu = parse_gerber(&bcu);
    assert!(!parsed_bcu.draws.is_empty(), "B_Cu must have track draws (D01)");
    assert!(!parsed_bcu.flashes.is_empty(), "B_Cu must have pad flashes (D03)");

    // 3. F_SilkS Layer: Silkscreen must have draws and no flashes
    let silk = export_gerber_layer(&board, Layer::FSilkscreen);
    let parsed_silk = parse_gerber(&silk);
    assert!(!parsed_silk.draws.is_empty(), "F_SilkS must have silkscreen draws (D01)");
    assert_eq!(parsed_silk.flashes.len(), 0, "F_SilkS should not have pad flashes");

    // 4. Edge_Cuts Layer: Must trace exactly the 4 border segments of the 65.0 x 32.0 mm rectangle
    let edge = export_gerber_layer(&board, Layer::EdgeCuts);
    let parsed_edge = parse_gerber(&edge);
    assert_eq!(parsed_edge.draws.len(), 4, "Edge.Cuts must have exactly 4 outline draw segments");
    assert_eq!(parsed_edge.flashes.len(), 0, "Edge.Cuts has 0 flashes");

    // Coordinate verification on Edge.Cuts: Check vertices 0.0 and 65.0, 32.0 at 10^6 scaling
    let mut edge_coords = HashSet::new();
    for draw in &parsed_edge.draws {
        edge_coords.insert((draw.start_x_nm, draw.start_y_nm));
        edge_coords.insert((draw.end_x_nm, draw.end_y_nm));
    }
    assert!(edge_coords.contains(&(0, 0)), "Outline contains (0, 0)");
    assert!(edge_coords.contains(&(65_000_000, 0)), "Outline contains (65mm, 0)");
    assert!(edge_coords.contains(&(65_000_000, 32_000_000)), "Outline contains (65mm, 32mm)");
    assert!(edge_coords.contains(&(0, 32_000_000)), "Outline contains (0, 32mm)");

    // 5. F_Mask and B_Mask: Solder mask openings
    let fmask = export_gerber_layer(&board, Layer::FMask);
    let parsed_fmask = parse_gerber(&fmask);
    assert!(!parsed_fmask.flashes.is_empty(), "F_Mask must have pad mask openings");

    let bmask = export_gerber_layer(&board, Layer::BMask);
    let parsed_bmask = parse_gerber(&bmask);
    assert!(!parsed_bmask.flashes.is_empty(), "B_Mask must have THT pin mask openings");
}

#[test]
fn test_excellon_drill_oracle_hits_and_deduplication() {
    let mut board = create_reference_board();
    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    };
    route_board(&mut board, &config).expect("Routing must succeed");

    let drl_content = export_excellon_drill(&board);
    let parsed = parse_drill(&drl_content);

    // Verify Tools Table
    assert_eq!(parsed.tools.len(), 2);
    // Tool 1: 0.400 mm for via drill holes
    assert_eq!(*parsed.tools.get(&1).unwrap(), 0.40);
    // Tool 2: 1.000 mm for standard 0.1" header pin holes (J2 and J3)
    assert_eq!(*parsed.tools.get(&2).unwrap(), 1.00);

    // Tool 1 hits: board.vias has 38 vias, with 2 duplicate pairs (coincident vias at net branch points)
    // The Excellon drill exporter deduplicates coincident drill hits down to 36 unique physical hole hits!
    let via_hits = parsed.hits_by_tool.get(&1).expect("Tool 1 via hits must exist");
    let expected: HashSet<_> = board.vias.iter().map(|v| ((v.at.x*1000.0).round() as i64, ((board.height-v.at.y)*1000.0).round() as i64)).collect();
    let actual: HashSet<_> = via_hits.iter().map(|v| ((v.x*1000.0).round() as i64, (v.y*1000.0).round() as i64)).collect();
    assert_eq!(actual, expected);

    // Tool 2 hits: J2 has 15 pins, J3 has 15 pins, and J1 USB-C has 4 THT shield tabs = 34 THT pins
    let tht_hits = parsed.hits_by_tool.get(&2).expect("Tool 2 THT hits must exist");
    assert_eq!(tht_hits.len(), 34, "Tool 2 hits must equal exactly 34 THT pins (30 header pins + 4 USB-C shield tabs)");

    // Total unique physical drill hits across the entire dev board: 36 vias + 34 THT = 70 hits
    let total_drill_hits = via_hits.len() + tht_hits.len();
    assert_eq!(total_drill_hits, expected.len()+34);

    // Deduplication invariant check: verify no two hits in Tool 1 or Tool 2 share the exact same location
    let mut unique_via_hits = HashSet::new();
    for hit in via_hits {
        let key = ((hit.x * 1000.0).round() as i64, (hit.y * 1000.0).round() as i64);
        assert!(unique_via_hits.insert(key), "Duplicate via drill hit detected at ({}, {})", hit.x, hit.y);
    }

    let mut unique_tht_hits = HashSet::new();
    for hit in tht_hits {
        let key = ((hit.x * 1000.0).round() as i64, (hit.y * 1000.0).round() as i64);
        assert!(unique_tht_hits.insert(key), "Duplicate THT drill hit detected at ({}, {})", hit.x, hit.y);
    }
}

// ============================================================================
// Task 3: Adversarial DRC Violation Injection Stress Tests
// ============================================================================

#[test]
fn test_drc_adversarial_injected_short_circuit_detection() {
    let mut board = create_reference_board();
    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    };
    route_board(&mut board, &config).expect("Routing must succeed");

    // Baseline: clean
    let base_drc = run_drc(&board, &config);
    assert_eq!(base_drc.short_circuits.len(), 0);

    // Inject deliberate short circuit between Net 1 (GND) and Net 2 (5V)
    let net_gnd = board.get_or_create_net("GND");
    let net_5v = board.get_or_create_net("5V");
    assert_ne!(net_gnd, net_5v);

    // Find a track of net_5v
    let t_5v = board.tracks.iter().find(|t| t.net == net_5v).expect("Net 5V track must exist");
    // Place a cross track of net_gnd right through it
    let cross_start = Point::new(t_5v.start.x, t_5v.start.y - 2.0);
    let cross_end = Point::new(t_5v.start.x, t_5v.start.y + 2.0);
    board.tracks.push(TrackSegment::new(cross_start, cross_end, 0.25, Layer::FCu, net_gnd));

    let challenged_drc = run_drc(&board, &config);
    assert!(
        !challenged_drc.short_circuits.is_empty(),
        "DRC must detect injected cross-net short circuit"
    );
    assert!(
        challenged_drc.short_circuits.iter().any(|v| v.kind == DrcViolationKind::ShortCircuit),
        "Violation kind must be ShortCircuit"
    );
}

#[test]
fn test_drc_adversarial_injected_clearance_infringement_detection() {
    let mut board = create_reference_board();
    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    };
    route_board(&mut board, &config).expect("Routing must succeed");

    let net_gnd = board.get_or_create_net("GND");
    let net_5v = board.get_or_create_net("5V");

    // Find a track of net_5v on F.Cu
    let t_5v = board.tracks.iter().find(|t| t.net == net_5v && t.layer == Layer::FCu).cloned().expect("5V F.Cu track");

    // Place a parallel track belonging to net_gnd with only 0.08 mm clearance (required 0.20 mm)
    // distance between centerlines = width + 0.08 = 0.25 + 0.08 = 0.33 mm
    let offset = 0.33;
    let (dx, dy) = if (t_5v.start.x - t_5v.end.x).abs() < 1e-5 {
        (offset, 0.0)
    } else {
        (0.0, offset)
    };
    let t_infringe = TrackSegment::new(
        Point::new(t_5v.start.x + dx, t_5v.start.y + dy),
        Point::new(t_5v.end.x + dx, t_5v.end.y + dy),
        0.25,
        Layer::FCu,
        net_gnd,
    );
    board.tracks.push(t_infringe);

    let drc = run_drc(&board, &config);
    assert!(
        !drc.clearance_violations.is_empty(),
        "DRC must detect 0.08mm clearance violation (when 0.20mm required)"
    );
    assert!(
        drc.clearance_violations.iter().any(|v| v.kind == DrcViolationKind::TraceTraceClearance),
        "Violation must be TraceTraceClearance"
    );
}

#[test]
fn test_drc_adversarial_injected_annular_ring_and_breakout_detection() {
    let mut board = create_reference_board();
    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    };

    // Subcase A: Annular ring too small (size 0.50, drill 0.40 -> ring 0.05 < 0.15 required)
    board.vias.push(Via::new(Point::new(30.0, 10.0), 0.50, 0.40, 1));
    let drc_a = run_drc(&board, &config);
    assert!(
        drc_a.annular_ring_violations.iter().any(|v| v.kind == DrcViolationKind::AnnularRingTooSmall),
        "DRC must detect undersized annular ring (0.05mm < 0.15mm)"
    );

    // Subcase B: Drill breakout (drill 0.60 >= size 0.50)
    board.vias.clear();
    board.vias.push(Via::new(Point::new(30.0, 10.0), 0.50, 0.60, 1));
    let drc_b = run_drc(&board, &config);
    assert!(
        drc_b.annular_ring_violations.iter().any(|v| v.kind == DrcViolationKind::DrillBreakout),
        "DRC must detect fatal drill breakout (drill >= size)"
    );
}

#[test]
fn test_drc_adversarial_injected_edge_keepout_and_boundary_detection() {
    let mut board = create_reference_board();
    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    };

    // Subcase A: Edge Keepout Violation (within board, but closer than MIN_EDGE_KEEPOUT = 0.40mm)
    // Board is (0, 0) to (65, 32).
    // Center at x = 0.25, width = 0.25 -> bbox.min.x = 0.125 >= 0.0, edge_dist = 0.125 < 0.40 mm
    board.tracks.push(TrackSegment::new(
        Point::new(0.25, 5.0),
        Point::new(0.25, 20.0),
        0.25,
        Layer::FCu,
        1,
    ));

    let drc_keepout = run_drc(&board, &config);
    assert!(
        drc_keepout.edge_keepout_violations.iter().any(|v| v.kind == DrcViolationKind::EdgeKeepoutViolation),
        "DRC must report EdgeKeepoutViolation when copper is within 0.40mm keepout"
    );

    // Subcase B: Outside Board Boundary (bbox.min.x < 0.0)
    board.tracks.clear();
    board.tracks.push(TrackSegment::new(
        Point::new(0.05, 5.0),
        Point::new(0.05, 20.0),
        0.25,
        Layer::FCu,
        1,
    ));

    let drc_boundary = run_drc(&board, &config);
    assert!(
        drc_boundary.edge_keepout_violations.iter().any(|v| v.kind == DrcViolationKind::OutsideBoardBoundary),
        "DRC must report OutsideBoardBoundary when copper extends past board edge"
    );
}

// ============================================================================
// Task 4: Dynamic Ratsnest Invalidation & Island Clustering Under Copper Connection
// ============================================================================

#[test]
fn test_dynamic_ratsnest_exact_island_clustering() {
    // Construct a board with 3 pads on Net 1
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_BUS");

    let p1 = Point::new(10.0, 10.0);
    let p2 = Point::new(20.0, 10.0);
    let p3 = Point::new(30.0, 10.0);

    let pad1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let pad2 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let pad3 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);

    board.add_footprint(Footprint::new("U1", "C", "P", p1, 0.0, vec![pad1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "C", "P", p2, 0.0, vec![pad2], Vec::new()));
    board.add_footprint(Footprint::new("U3", "C", "P", p3, 0.0, vec![pad3], Vec::new()));

    // 1. Unrouted: 2 ratsnest lines (p1-p2, p2-p3)
    let rn_unrouted = compute_ratsnest(&board);
    assert_eq!(rn_unrouted.len(), 2, "3 pins produce 2 ratsnest lines");

    // 2. Connect p1 and p2: clusters island 1, leaving 1 ratsnest line (p2-p3)
    board.add_track(TrackSegment::new(p1, p2, 0.25, Layer::FCu, 1));
    let rn_half = compute_ratsnest(&board);
    assert_eq!(rn_half.len(), 1, "Connecting 2 pins reduces ratsnest to 1 line");

    // 3. Connect p2 and p3: all pins in single copper island -> 0 lines
    board.add_track(TrackSegment::new(p2, p3, 0.25, Layer::FCu, 1));
    let rn_full = compute_ratsnest(&board);
    assert_eq!(rn_full.len(), 0, "Connecting all pins reduces ratsnest to 0 lines");

    // 4. Invalidation: sever p2-p3 track -> ratsnest line dynamically reappears!
    board.tracks.pop();
    let rn_severed = compute_ratsnest(&board);
    assert_eq!(rn_severed.len(), 1, "Severing track dynamically regenerates ratsnest line");
}

// ============================================================================
// Task 5: Router Parameter Validation & Boundary Edge Cases
// ============================================================================

#[test]
fn test_router_config_invalid_parameter_rejections() {
    let mut board = create_reference_board();

    // Zero or negative grid resolution
    let bad_cfg1 = RouterConfig {
        grid_resolution: 0.0,
        default_trace_width: 0.25,
        via_diameter: 0.8,
        via_drill: 0.4,
        clearance: 0.2,
    };
    assert!(route_board(&mut board, &bad_cfg1).is_err(), "Must reject grid_resolution = 0.0");

    let bad_cfg1_neg = RouterConfig {
        grid_resolution: -0.25,
        default_trace_width: 0.25,
        via_diameter: 0.8,
        via_drill: 0.4,
        clearance: 0.2,
    };
    assert!(route_board(&mut board, &bad_cfg1_neg).is_err(), "Must reject grid_resolution < 0.0");

    // Zero or negative trace width
    let bad_cfg2 = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.0,
        via_diameter: 0.8,
        via_drill: 0.4,
        clearance: 0.2,
    };
    assert!(route_board(&mut board, &bad_cfg2).is_err(), "Must reject trace_width = 0.0");

    let bad_cfg2_neg = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: -0.1,
        via_diameter: 0.8,
        via_drill: 0.4,
        clearance: 0.2,
    };
    assert!(route_board(&mut board, &bad_cfg2_neg).is_err(), "Must reject negative trace width");
}
