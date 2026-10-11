//! Empirical Challenger 2 Adversarial Test Suite for Milestone 4
//! RS-274X Gerber Exporter & Excellon NC Drill Exporter
//!
//! Areas Covered:
//! 1. All 6 Standard Fabrication Layers (F_Cu, B_Cu, F_SilkS, Edge_Cuts, F_Mask, B_Mask)
//! 2. Gerber Aperture Table (D10+ sequential indexing, exact dimension matching, pad flash binding)
//! 3. Coordinate Scaling (10^6 integer coordinates, nanometer resolution, leading-zero suppression, 4-quadrant handling)
//! 4. Excellon Drill Exporter (Tool sorting by diameter ascending, tool change grouping, hit coordinate precision)
//! 5. Complex & Boundary Scenarios (Empty boards, dense boards with 100s of components, mixed SMD/THT, rotated components)

use netlist_shatter_viz::pcb::*;
use std::collections::{BTreeMap, HashSet};

// ============================================================================
// Independent RS-274X & Excellon Parsing Oracles
// ============================================================================

#[derive(Clone, Debug, PartialEq)]
enum ParsedAperture {
    Circle { dia: f64 },
    Rect { w: f64, h: f64 },
    Obround { w: f64, h: f64 },
}

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
            // Format: <dcode><Type>,<params>
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
            // Aperture selection, e.g. D10*
            let code_str = &line[1..line.len() - 1];
            let code: u32 = code_str.parse().expect("dcode selection");
            current_dcode = Some(code);
        } else if line.contains("D03*") {
            // Flash command, e.g. X1000000Y2000000D03*
            let dcode = current_dcode.expect("Active aperture required for D03 flash");
            let (x, y) = parse_xy_nm(line);
            flashes.push(ParsedFlash { dcode, x_nm: x, y_nm: y });
        } else if line.contains("D02*") {
            // Move command, e.g. X1000000Y2000000D02*
            let (x, y) = parse_xy_nm(line);
            pending_move = Some((x, y));
        } else if line.contains("D01*") {
            // Interpolate / draw command, e.g. X3000000Y4000000D01*
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
    pub tools: BTreeMap<u32, f64>, // tool_num -> diameter_mm
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
                // T01C0.400
                let c_idx = line.find('C').unwrap();
                let tool_num: u32 = line[1..c_idx].parse().expect("tool number");
                let dia: f64 = line[c_idx + 1..].parse().expect("tool diameter");
                tools.insert(tool_num, dia);
            }
        } else {
            if line.starts_with('T') && !line.contains('C') {
                // Tool selection, e.g. T01
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

// ============================================================================
// 1. ALL 6 STANDARD FABRICATION LAYERS VERIFICATION & ADVERSARIAL STRESS
// ============================================================================

#[test]
fn test_all_6_layers_filenames_and_presence() {
    let board = PcbBoard::new(50.0, 40.0);
    let all_files = export_all_gerbers_and_drill(&board);

    let expected_files = [
        "F_Cu.gbr",
        "B_Cu.gbr",
        "F_SilkS.gbr",
        "Edge_Cuts.gbr",
        "F_Mask.gbr",
        "B_Mask.gbr",
        "drill.drl",
    ];

    assert_eq!(all_files.len(), 7, "Must export exactly 6 Gerber layers and 1 Excellon drill file");
    for &expected in &expected_files {
        assert!(all_files.contains_key(expected), "Missing expected export file: {}", expected);
    }
}

#[test]
fn test_empty_board_all_6_layers_compliance() {
    let board = PcbBoard::new(60.0, 30.0);
    let layers = [
        Layer::FCu,
        Layer::BCu,
        Layer::FSilkscreen,
        Layer::EdgeCuts,
        Layer::FMask,
        Layer::BMask,
    ];

    for layer in layers {
        let gbr_content = export_gerber_layer(&board, layer);
        let parsed = parse_gerber(&gbr_content);

        assert!(parsed.has_format, "Layer {:?} missing %FSLAX46Y46*%", layer);
        assert!(parsed.has_momm, "Layer {:?} missing %MOMM*%", layer);
        assert!(parsed.has_lpd, "Layer {:?} missing %LPD*%", layer);
        assert!(parsed.ends_with_m02, "Layer {:?} must end with M02*", layer);

        if layer == Layer::EdgeCuts {
            // EdgeCuts must still contain the 4 boundary segments
            assert_eq!(parsed.draws.len(), 4, "Edge.Cuts on empty board must draw 4 boundary lines");
            assert_eq!(parsed.apertures.len(), 1, "Edge.Cuts needs 1 aperture for boundary lines");
        } else {
            // All other layers must be empty of geometry
            assert_eq!(parsed.flashes.len(), 0, "Layer {:?} must have 0 flashes on empty board", layer);
            assert_eq!(parsed.draws.len(), 0, "Layer {:?} must have 0 draws on empty board", layer);
            assert_eq!(parsed.apertures.len(), 0, "Layer {:?} must have 0 apertures on empty board", layer);
        }
    }

    let drl_content = export_excellon_drill(&board);
    let parsed_drl = parse_drill(&drl_content);
    assert!(parsed_drl.starts_with_m48);
    assert!(parsed_drl.has_metric);
    assert!(parsed_drl.ends_with_m30);
    assert_eq!(parsed_drl.tools.len(), 0);
    assert_eq!(parsed_drl.hits_by_tool.len(), 0);
}

#[test]
fn test_layer_isolation_smd_top_vs_bottom() {
    let mut board = PcbBoard::new(50.0, 50.0);

    // Footprint on F.Cu
    let top_pad = Pad::new_smd("1", "VCC", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 2.0 }, Layer::FCu);
    let top_fp = Footprint::new("R_TOP", "10k", "0805", Point::new(10.0, 10.0), 0.0, vec![top_pad], vec![]);
    board.add_footprint(top_fp);

    // Footprint on B.Cu
    let bot_pad = Pad::new_smd("1", "GND", Point::new(0.0, 0.0), PadShape::Rect { width: 1.5, height: 0.5 }, Layer::BCu);
    let bot_fp = Footprint::new("R_BOT", "10k", "0805", Point::new(30.0, 30.0), 0.0, vec![bot_pad], vec![]);
    board.add_footprint(bot_fp);

    // Verify F_Cu
    let fcu = parse_gerber(&export_gerber_layer(&board, Layer::FCu));
    assert_eq!(fcu.flashes.len(), 1, "F_Cu must have exactly 1 flash for top pad");
    assert_eq!(fcu.flashes[0].x_nm, 10_000_000);
    assert_eq!(fcu.flashes[0].y_nm, 40_000_000);

    // Verify B_Cu
    let bcu = parse_gerber(&export_gerber_layer(&board, Layer::BCu));
    assert_eq!(bcu.flashes.len(), 1, "B_Cu must have exactly 1 flash for bottom pad");
    assert_eq!(bcu.flashes[0].x_nm, 30_000_000);
    assert_eq!(bcu.flashes[0].y_nm, 20_000_000);

    // Verify F_Mask
    let fmask = parse_gerber(&export_gerber_layer(&board, Layer::FMask));
    assert_eq!(fmask.flashes.len(), 1, "F_Mask must have exactly 1 flash for top pad mask opening");
    assert_eq!(fmask.flashes[0].x_nm, 10_000_000);
    assert_eq!(fmask.flashes[0].y_nm, 40_000_000);

    // Verify B_Mask
    let bmask = parse_gerber(&export_gerber_layer(&board, Layer::BMask));
    assert_eq!(bmask.flashes.len(), 1, "B_Mask must have exactly 1 flash for bottom pad mask opening");
    assert_eq!(bmask.flashes[0].x_nm, 30_000_000);
    assert_eq!(bmask.flashes[0].y_nm, 20_000_000);
}

#[test]
fn test_tht_pads_present_on_both_copper_and_both_mask_layers() {
    let mut board = PcbBoard::new(50.0, 50.0);

    // Single THT pad at (20.0, 25.0)
    let tht_pad = Pad::new_tht("1", "PIN1", Point::new(0.0, 0.0), PadShape::Circle { diameter: 1.8 }, 1.0, 1.8);
    let fp = Footprint::new("J1", "CONN", "1PIN", Point::new(20.0, 25.0), 0.0, vec![tht_pad], vec![]);
    board.add_footprint(fp);

    let check_layers = [Layer::FCu, Layer::BCu, Layer::FMask, Layer::BMask];
    for layer in check_layers {
        let gbr = parse_gerber(&export_gerber_layer(&board, layer));
        assert_eq!(gbr.flashes.len(), 1, "THT pad must be flashed on layer {:?}", layer);
        assert_eq!(gbr.flashes[0].x_nm, 20_000_000);
        assert_eq!(gbr.flashes[0].y_nm, 25_000_000);

        let ap = gbr.apertures.get(&gbr.flashes[0].dcode).expect("Aperture defined");
        assert_eq!(*ap, ParsedAperture::Circle { dia: 1.8 });
    }
}

#[test]
fn test_edge_cuts_closed_perimeter_and_explicit_tracks() {
    let mut board = PcbBoard::new(80.0, 50.0);
    // Add an interior milling slot on EdgeCuts layer
    board.add_track(TrackSegment::new(Point::new(10.0, 10.0), Point::new(20.0, 10.0), 1.0, Layer::EdgeCuts, 0));

    let edge_gbr = parse_gerber(&export_gerber_layer(&board, Layer::EdgeCuts));

    // 4 perimeter lines + 1 explicit slot track = 5 draws
    assert_eq!(edge_gbr.draws.len(), 5);

    // Verify perimeter boundary loop is closed:
    // (0,0)->(80,0)->(80,50)->(0,50)->(0,0)
    let mut perimeter_starts = HashSet::new();
    let mut perimeter_ends = HashSet::new();

    for d in &edge_gbr.draws[0..4] {
        perimeter_starts.insert((d.start_x_nm, d.start_y_nm));
        perimeter_ends.insert((d.end_x_nm, d.end_y_nm));
    }

    assert_eq!(perimeter_starts, perimeter_ends, "Perimeter draws must form a closed cycle");
    assert!(perimeter_starts.contains(&(0, 0)));
    assert!(perimeter_starts.contains(&(80_000_000, 0)));
    assert!(perimeter_starts.contains(&(80_000_000, 50_000_000)));
    assert!(perimeter_starts.contains(&(0, 50_000_000)));
}

// ============================================================================
// 2. GERBER APERTURE DEFINITIONS & FLASHED PAD MATCHING
// ============================================================================

#[test]
fn test_aperture_table_d10_sequential_indexing_no_gaps() {
    let mut board = PcbBoard::new(100.0, 100.0);

    // Create 15 distinct apertures (various shapes and dimensions)
    let pad_shapes = [
        PadShape::Circle { diameter: 0.5 },
        PadShape::Circle { diameter: 0.8 },
        PadShape::Circle { diameter: 1.2 },
        PadShape::Circle { diameter: 1.6 },
        PadShape::Rect { width: 0.5, height: 1.0 },
        PadShape::Rect { width: 1.0, height: 0.5 },
        PadShape::Rect { width: 1.5, height: 1.5 },
        PadShape::Rect { width: 2.0, height: 0.8 },
        PadShape::Obround { width: 1.2, height: 0.6 },
        PadShape::Obround { width: 0.6, height: 1.2 },
        PadShape::Obround { width: 2.5, height: 1.0 },
    ];

    let mut pads = Vec::new();
    for (i, &shape) in pad_shapes.iter().enumerate() {
        pads.push(Pad::new_smd(
            format!("{}", i + 1),
            "PAD",
            Point::new((i as f64) * 4.0, 0.0),
            shape,
            Layer::FCu,
        ));
    }
    board.add_footprint(Footprint::new("U1", "TEST_IC", "CUSTOM", Point::new(10.0, 10.0), 0.0, pads, vec![]));

    let fcu = parse_gerber(&export_gerber_layer(&board, Layer::FCu));
    assert_eq!(fcu.apertures.len(), 11, "Must contain exactly 11 unique apertures");

    let mut expected_dcode = 10u32;
    for (&dcode, _) in &fcu.apertures {
        assert_eq!(dcode, expected_dcode, "D-codes must be strictly contiguous starting at 10");
        expected_dcode += 1;
    }

    // Verify all flashes reference an aperture that exists in the table
    for flash in &fcu.flashes {
        assert!(fcu.apertures.contains_key(&flash.dcode), "Flash references undefined aperture D{}", flash.dcode);
    }
}

#[test]
fn test_aperture_exact_dimension_matching_and_pad_binding() {
    let mut board = PcbBoard::new(80.0, 80.0);

    let p1 = Pad::new_smd("1", "P1", Point::new(-5.0, 0.0), PadShape::Rect { width: 1.25, height: 0.75 }, Layer::FCu);
    let p2 = Pad::new_smd("2", "P2", Point::new(5.0, 0.0), PadShape::Obround { width: 2.20, height: 1.10 }, Layer::FCu);
    let p3 = Pad::new_tht("3", "P3", Point::new(0.0, 5.0), PadShape::Circle { diameter: 1.65 }, 0.8, 1.65);

    let fp = Footprint::new("MOD1", "SAMPLE", "PKG", Point::new(30.0, 30.0), 0.0, vec![p1, p2, p3], vec![]);
    board.add_footprint(fp);

    let fcu = parse_gerber(&export_gerber_layer(&board, Layer::FCu));

    // Must have 3 flashes
    assert_eq!(fcu.flashes.len(), 3);

    // Map each flash coordinate back to expected pad
    for flash in &fcu.flashes {
        let ap = fcu.apertures.get(&flash.dcode).unwrap();
        if flash.x_nm == 25_000_000 && flash.y_nm == 50_000_000 {
            // p1: Rect 1.25 x 0.75
            assert_eq!(*ap, ParsedAperture::Rect { w: 1.25, h: 0.75 });
        } else if flash.x_nm == 35_000_000 && flash.y_nm == 50_000_000 {
            // p2: Obround 2.20 x 1.10
            assert_eq!(*ap, ParsedAperture::Obround { w: 2.20, h: 1.10 });
        } else if flash.x_nm == 30_000_000 && flash.y_nm == 45_000_000 {
            // p3: Circle 1.65
            assert_eq!(*ap, ParsedAperture::Circle { dia: 1.65 });
        } else {
            panic!("Unexpected flash at X={} Y={}", flash.x_nm, flash.y_nm);
        }
    }
}

#[test]
fn test_aperture_rotation_transposition_0_90_180_270() {
    let mut board = PcbBoard::new(100.0, 100.0);

    let base_pad = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 2.4, height: 0.8 }, Layer::FCu);

    // 4 footprints rotated 0°, 90°, 180°, 270°
    let fp0 = Footprint::new("R0", "V", "P", Point::new(10.0, 20.0), 0.0, vec![base_pad.clone()], vec![]);
    let fp90 = Footprint::new("R90", "V", "P", Point::new(30.0, 20.0), 90.0, vec![base_pad.clone()], vec![]);
    let fp180 = Footprint::new("R180", "V", "P", Point::new(50.0, 20.0), 180.0, vec![base_pad.clone()], vec![]);
    let fp270 = Footprint::new("R270", "V", "P", Point::new(70.0, 20.0), 270.0, vec![base_pad.clone()], vec![]);

    board.add_footprint(fp0);
    board.add_footprint(fp90);
    board.add_footprint(fp180);
    board.add_footprint(fp270);

    let fcu = parse_gerber(&export_gerber_layer(&board, Layer::FCu));

    // There should be exactly TWO unique apertures: 2.4x0.8 and 0.8x2.4
    assert_eq!(fcu.apertures.len(), 2, "0° and 180° share one aperture; 90° and 270° share transposed aperture");

    let ap_unrotated = fcu.apertures.values().find(|&&ref ap| *ap == ParsedAperture::Rect { w: 2.4, h: 0.8 });
    let ap_transposed = fcu.apertures.values().find(|&&ref ap| *ap == ParsedAperture::Rect { w: 0.8, h: 2.4 });

    assert!(ap_unrotated.is_some(), "Must have unrotated 2.4x0.8 aperture");
    assert!(ap_transposed.is_some(), "Must have transposed 0.8x2.4 aperture");

    // Match flashes
    for flash in &fcu.flashes {
        let ap = fcu.apertures.get(&flash.dcode).unwrap();
        if flash.x_nm == 10_000_000 || flash.x_nm == 50_000_000 {
            assert_eq!(*ap, ParsedAperture::Rect { w: 2.4, h: 0.8 }, "0° and 180° must use unrotated aperture");
        } else if flash.x_nm == 30_000_000 || flash.x_nm == 70_000_000 {
            assert_eq!(*ap, ParsedAperture::Rect { w: 0.8, h: 2.4 }, "90° and 270° must use transposed aperture");
        }
    }
}

#[test]
fn test_aperture_deduplication_stress() {
    let mut board = PcbBoard::new(100.0, 100.0);

    // 100 identical pads
    let mut pads = Vec::new();
    for i in 0..100 {
        pads.push(Pad::new_smd(
            format!("{}", i + 1),
            "PAD",
            Point::new((i % 10) as f64 * 3.0, (i / 10) as f64 * 3.0),
            PadShape::Rect { width: 1.2, height: 0.6 },
            Layer::FCu,
        ));
    }
    board.add_footprint(Footprint::new("U1", "IC", "PKG", Point::new(10.0, 10.0), 0.0, pads, vec![]));

    let fcu = parse_gerber(&export_gerber_layer(&board, Layer::FCu));
    assert_eq!(fcu.apertures.len(), 1, "100 identical pads must produce exactly 1 aperture definition");
    assert_eq!(fcu.flashes.len(), 100, "Must flash 100 times");
    for flash in &fcu.flashes {
        assert_eq!(flash.dcode, 10, "All flashes must use D10");
    }
}

// ============================================================================
// 3. COORDINATE SCALING & QUADRANT HANDLING
// ============================================================================

#[test]
fn test_coordinate_scaling_sub_micron_precision() {
    let mut board = PcbBoard::new(100.0, 100.0);
    // Nanometer level coordinate: 12.345678 mm -> 12,345,678 nm
    board.add_track(TrackSegment::new(
        Point::new(0.000001, 0.000002),
        Point::new(12.345678, 45.678901),
        0.25,
        Layer::FCu,
        1,
    ));

    let fcu = parse_gerber(&export_gerber_layer(&board, Layer::FCu));
    assert_eq!(fcu.draws.len(), 1);
    assert_eq!(fcu.draws[0].start_x_nm, 1);
    assert_eq!(fcu.draws[0].start_y_nm, 99_999_998);
    assert_eq!(fcu.draws[0].end_x_nm, 12_345_678);
    assert_eq!(fcu.draws[0].end_y_nm, 54_321_099);
}

#[test]
fn test_leading_zero_suppression_and_zero_values() {
    let mut board = PcbBoard::new(50.0, 50.0);
    // Pad at origin (0.0, 0.0)
    let p0 = Pad::new_smd("1", "P", Point::new(0.0, 0.0), PadShape::Circle { diameter: 0.5 }, Layer::FCu);
    board.add_footprint(Footprint::new("U1", "V", "P", Point::new(0.0, 0.0), 0.0, vec![p0], vec![]));

    // Pad at 0.05 mm = 50,000 nm
    let p1 = Pad::new_smd("2", "P", Point::new(0.05, 0.05), PadShape::Circle { diameter: 0.5 }, Layer::FCu);
    board.add_footprint(Footprint::new("U2", "V", "P", Point::new(0.0, 0.0), 0.0, vec![p1], vec![]));

    let raw_gbr = export_gerber_layer(&board, Layer::FCu);

    // Verify leading-zero suppression:
    // Should contain "X0Y50000000D03*"
    assert!(raw_gbr.contains("X0Y50000000D03*"), "Origin flash must be X0Y50000000D03*");

    // Should contain "X50000Y49950000D03*", NOT "X000050000" or similar
    assert!(raw_gbr.contains("X50000Y49950000D03*"), "0.05mm must be formatted with leading zeros omitted: X50000Y49950000D03*");
    assert!(!raw_gbr.contains("X000"), "Leading zeros must be suppressed");
}

#[test]
fn test_four_quadrants_and_negative_coordinates() {
    let mut board = PcbBoard::new(100.0, 100.0);

    // Elements in all four quadrants centered around origin (0, 0):
    // Q1: (+10.5, +20.25)
    // Q2: (-10.5, +20.25)
    // Q3: (-10.5, -20.25)
    // Q4: (+10.5, -20.25)
    let pad_shape = PadShape::Circle { diameter: 0.6 };

    let fp_q1 = Footprint::new("Q1", "V", "P", Point::new(10.5, 20.25), 0.0, vec![Pad::new_smd("1", "", Point::zero(), pad_shape, Layer::FCu)], vec![]);
    let fp_q2 = Footprint::new("Q2", "V", "P", Point::new(-10.5, 20.25), 0.0, vec![Pad::new_smd("1", "", Point::zero(), pad_shape, Layer::FCu)], vec![]);
    let fp_q3 = Footprint::new("Q3", "V", "P", Point::new(-10.5, -20.25), 0.0, vec![Pad::new_smd("1", "", Point::zero(), pad_shape, Layer::FCu)], vec![]);
    let fp_q4 = Footprint::new("Q4", "V", "P", Point::new(10.5, -20.25), 0.0, vec![Pad::new_smd("1", "", Point::zero(), pad_shape, Layer::FCu)], vec![]);

    board.add_footprint(fp_q1);
    board.add_footprint(fp_q2);
    board.add_footprint(fp_q3);
    board.add_footprint(fp_q4);

    // Track traversing diagonally across all 4 quadrants: (-30, -30) to (+30, +30)
    board.add_track(TrackSegment::new(Point::new(-30.0, -30.0), Point::new(30.0, 30.0), 0.25, Layer::FCu, 1));

    let raw_gbr = export_gerber_layer(&board, Layer::FCu);
    let parsed = parse_gerber(&raw_gbr);

    assert_eq!(parsed.flashes.len(), 4);

    let mut flash_coords = HashSet::new();
    for f in &parsed.flashes {
        flash_coords.insert((f.x_nm, f.y_nm));
    }

    assert!(flash_coords.contains(&(10_500_000, 79_750_000)), "Q1 flash (+,+)");
    assert!(flash_coords.contains(&(-10_500_000, 79_750_000)), "Q2 flash (-,+)");
    assert!(flash_coords.contains(&(-10_500_000, 120_250_000)), "Q3 flash (-,-)");
    assert!(flash_coords.contains(&(10_500_000, 120_250_000)), "Q4 flash (+,-)");

    // Raw Gerber syntax checks for minus signs
    assert!(raw_gbr.contains("X-10500000Y79750000D03*"));
    assert!(raw_gbr.contains("X-10500000Y120250000D03*"));
    assert!(raw_gbr.contains("X10500000Y120250000D03*"));
    assert!(raw_gbr.contains("X-30000000Y130000000D02*"));
    assert!(raw_gbr.contains("X30000000Y70000000D01*"));
}

// ============================================================================
// 4. EXCELLON DRILL EXPORTER VERIFICATION & ADVERSARIAL STRESS
// ============================================================================

#[test]
fn test_excellon_tool_definition_sorting_ascending() {
    let mut board = PcbBoard::new(100.0, 100.0);

    // Insert drill holes out of order
    board.add_via(Via::new(Point::new(10.0, 10.0), 0.8, 0.4, 1));   // drill 0.4
    board.add_via(Via::new(Point::new(20.0, 10.0), 1.2, 0.8, 1));   // drill 0.8
    board.add_via(Via::new(Point::new(30.0, 10.0), 0.6, 0.25, 1));  // drill 0.25
    board.add_via(Via::new(Point::new(40.0, 10.0), 1.8, 1.2, 1));   // drill 1.2
    board.add_via(Via::new(Point::new(50.0, 10.0), 1.0, 0.5, 1));   // drill 0.5

    let drl = export_excellon_drill(&board);
    let parsed = parse_drill(&drl);

    assert_eq!(parsed.tools.len(), 5);

    // Tools must be sorted ascending by diameter:
    // T01: 0.250, T02: 0.400, T03: 0.500, T04: 0.800, T05: 1.200
    let mut prev_dia = 0.0;
    for (&tool_num, &dia) in &parsed.tools {
        assert!(dia > prev_dia, "Tool diameters must be strictly increasing: T{} has {} <= previous {}", tool_num, dia, prev_dia);
        prev_dia = dia;
    }

    assert_eq!(*parsed.tools.get(&1).unwrap(), 0.250);
    assert_eq!(*parsed.tools.get(&2).unwrap(), 0.400);
    assert_eq!(*parsed.tools.get(&3).unwrap(), 0.500);
    assert_eq!(*parsed.tools.get(&4).unwrap(), 0.800);
    assert_eq!(*parsed.tools.get(&5).unwrap(), 1.200);
}

#[test]
fn test_excellon_tool_changes_and_hit_grouping() {
    let mut board = PcbBoard::new(100.0, 100.0);

    // 3 holes with drill 0.3, 3 holes with drill 0.6, interleaved
    board.add_via(Via::new(Point::new(10.0, 10.0), 0.6, 0.3, 1));
    board.add_via(Via::new(Point::new(20.0, 20.0), 1.0, 0.6, 1));
    board.add_via(Via::new(Point::new(15.0, 10.0), 0.6, 0.3, 1));
    board.add_via(Via::new(Point::new(25.0, 20.0), 1.0, 0.6, 1));
    board.add_via(Via::new(Point::new(12.0, 10.0), 0.6, 0.3, 1));
    board.add_via(Via::new(Point::new(22.0, 20.0), 1.0, 0.6, 1));

    let drl = export_excellon_drill(&board);

    // Count occurrences of tool selection directives "T01" and "T02" in the body
    let body = &drl[drl.find('%').unwrap()..];
    let t01_count = body.lines().filter(|l| l.trim() == "T01").count();
    let t02_count = body.lines().filter(|l| l.trim() == "T02").count();

    assert_eq!(t01_count, 1, "T01 must be selected exactly once, grouping all hits");
    assert_eq!(t02_count, 1, "T02 must be selected exactly once, grouping all hits");

    let parsed = parse_drill(&drl);
    assert_eq!(parsed.hits_by_tool.get(&1).unwrap().len(), 3);
    assert_eq!(parsed.hits_by_tool.get(&2).unwrap().len(), 3);
}

#[test]
fn test_excellon_drill_hit_coordinate_accuracy_and_sorting() {
    let mut board = PcbBoard::new(100.0, 100.0);

    // Hits for tool 0.5mm with various coordinates
    let test_points = [
        Point::new(30.0, 40.0),
        Point::new(10.0, 40.0),
        Point::new(20.0, 10.0),
        Point::new(5.0, 10.0),
        Point::new(50.0, 60.0),
    ];

    for pt in test_points {
        board.add_via(Via::new(pt, 1.0, 0.5, 1));
    }

    let drl = export_excellon_drill(&board);
    let parsed = parse_drill(&drl);

    let hits = parsed.hits_by_tool.get(&1).expect("Hits for T01");
    assert_eq!(hits.len(), 5);

    // Hits must be sorted primary Y ascending, secondary X ascending
    for i in 0..hits.len() - 1 {
        let cur = hits[i];
        let next = hits[i + 1];
        let ord = cur.y.partial_cmp(&next.y).unwrap().then_with(|| cur.x.partial_cmp(&next.x).unwrap());
        assert!(ord == std::cmp::Ordering::Less || ord == std::cmp::Ordering::Equal,
            "Hits must be sorted by Y then X: hit[{}]={:?} vs hit[{}]={:?}", i, cur, i+1, next);
    }

    let expected = [(50.0,40.0),(10.0,60.0),(30.0,60.0),(5.0,90.0),(20.0,90.0)];
    for (hit,(x,y)) in hits.iter().zip(expected) {
        assert!((hit.x-x).abs()<1e-3 && (hit.y-y).abs()<1e-3);
    }
}

#[test]
fn test_drill_hole_alignment_with_gerber_flashes() {
    let mut board = PcbBoard::new(100.0, 100.0);

    // Footprint rotated 90° with off-center THT pad
    let tht_pad = Pad::new_tht("1", "PIN", Point::new(2.54, 1.27), PadShape::Circle { diameter: 1.6 }, 0.8, 1.6);
    let fp = Footprint::new("J1", "HEADER", "1PIN", Point::new(40.0, 50.0), 90.0, vec![tht_pad], vec![]);
    board.add_footprint(fp);

    // Also a via
    board.add_via(Via::new(Point::new(60.0, 70.0), 0.8, 0.4, 1));

    let fcu = parse_gerber(&export_gerber_layer(&board, Layer::FCu));
    let drl = parse_drill(&export_excellon_drill(&board));

    // Pad world position:
    // (40.0, 50.0) + (2.54, 1.27).rotate(90.0) = (40.0 - 1.27, 50.0 + 2.54) = (38.73, 52.54)
    let expected_pad_pt = Point::new(38.73, 47.46);
    let expected_via_pt = Point::new(60.0, 30.0);

    // Verify Gerber flashes exist at these exact coordinates
    let mut gbr_points = Vec::new();
    for f in &fcu.flashes {
        gbr_points.push(Point::new(f.x_nm as f64 / 1_000_000.0, f.y_nm as f64 / 1_000_000.0));
    }

    assert!(gbr_points.iter().any(|pt| (pt.x - expected_pad_pt.x).abs() < 1e-4 && (pt.y - expected_pad_pt.y).abs() < 1e-4),
        "Gerber must flash THT pad at rotated world coordinate (38.73, 52.54)");
    assert!(gbr_points.iter().any(|pt| (pt.x - expected_via_pt.x).abs() < 1e-4 && (pt.y - expected_via_pt.y).abs() < 1e-4),
        "Gerber must flash Via at (60.0, 70.0)");

    // Verify Drill hits exist at these exact coordinates
    let all_drill_hits: Vec<Point> = drl.hits_by_tool.values().flat_map(|v| v.iter().copied()).collect();
    assert!(all_drill_hits.iter().any(|pt| (pt.x - expected_pad_pt.x).abs() < 1e-3 && (pt.y - expected_pad_pt.y).abs() < 1e-3),
        "Drill file must hit THT pad at rotated world coordinate (38.73, 52.54)");
    assert!(all_drill_hits.iter().any(|pt| (pt.x - expected_via_pt.x).abs() < 1e-3 && (pt.y - expected_via_pt.y).abs() < 1e-3),
        "Drill file must hit Via at (60.0, 70.0)");
}

// ============================================================================
// 5. DENSE BOARDS, MIXED SMD/THT, & EDGE CASES
// ============================================================================

#[test]
fn test_dense_board_stress_hundreds_of_elements() {
    let mut board = PcbBoard::new(100.0, 100.0);

    // Generate a dense grid of 64 ICs (8x8 grid), each with 4 pads
    for row in 0..8 {
        for col in 0..8 {
            let ref_des = format!("U_{}_{}", row, col);
            let pos = Point::new(10.0 + col as f64 * 10.0, 10.0 + row as f64 * 10.0);
            let p1 = Pad::new_smd("1", "VCC", Point::new(-1.0, -1.0), PadShape::Rect { width: 0.8, height: 0.5 }, Layer::FCu);
            let p2 = Pad::new_smd("2", "GND", Point::new(1.0, -1.0), PadShape::Rect { width: 0.8, height: 0.5 }, Layer::FCu);
            let p3 = Pad::new_smd("3", "IN", Point::new(-1.0, 1.0), PadShape::Rect { width: 0.8, height: 0.5 }, Layer::FCu);
            let p4 = Pad::new_smd("4", "OUT", Point::new(1.0, 1.0), PadShape::Rect { width: 0.8, height: 0.5 }, Layer::FCu);

            let silk = vec![
                (Point::new(-2.0, -2.0), Point::new(2.0, -2.0)),
                (Point::new(2.0, -2.0), Point::new(2.0, 2.0)),
                (Point::new(2.0, 2.0), Point::new(-2.0, 2.0)),
                (Point::new(-2.0, 2.0), Point::new(-2.0, -2.0)),
            ];

            let fp = Footprint::new(ref_des, "LOGIC", "SOIC-4", pos, ((row + col) * 90 % 360) as f64, vec![p1, p2, p3, p4], silk);
            board.add_footprint(fp);

            // Connect with tracks and vias
            board.add_track(TrackSegment::new(pos, pos + Point::new(5.0, 0.0), 0.25, Layer::FCu, 1));
            board.add_track(TrackSegment::new(pos + Point::new(5.0, 0.0), pos + Point::new(5.0, 5.0), 0.25, Layer::BCu, 1));
            board.add_via(Via::new(pos + Point::new(5.0, 0.0), 0.8, 0.4, 1));
        }
    }

    let all_files = export_all_gerbers_and_drill(&board);
    assert_eq!(all_files.len(), 7);

    // Verify F_Cu
    let fcu = parse_gerber(all_files.get("F_Cu.gbr").unwrap());
    assert_eq!(fcu.flashes.len(), 64 * 4 + 64, "64 ICs * 4 pads + 64 vias = 320 flashes on F_Cu");
    assert_eq!(fcu.draws.len(), 64, "64 tracks on F_Cu");

    // Verify B_Cu
    let bcu = parse_gerber(all_files.get("B_Cu.gbr").unwrap());
    assert_eq!(bcu.flashes.len(), 64, "64 vias on B_Cu");
    assert_eq!(bcu.draws.len(), 64, "64 tracks on B_Cu");

    // Verify F_SilkS
    let silk = parse_gerber(all_files.get("F_SilkS.gbr").unwrap());
    assert_eq!(silk.draws.len(), 64 * 4, "64 ICs * 4 silkscreen lines = 256 silkscreen draws");

    // Verify Drill
    let drl = parse_drill(all_files.get("drill.drl").unwrap());
    assert_eq!(drl.tools.len(), 1, "All 64 vias share drill diameter 0.4mm");
    assert_eq!(drl.hits_by_tool.get(&1).unwrap().len(), 64, "64 drill hits");
}

#[test]
fn test_mixed_smd_and_tht_full_board() {
    let mut board = PcbBoard::new(60.0, 35.0);

    // SMD ATmega (TQFP-32)
    let mcu = footprint::tqfp32("U1", "ATmega328P").at_position(Point::new(30.0, 17.5));
    // SMD SOT-223 regulator
    let reg = footprint::sot223("U2", "AMS1117-3.3").at_position(Point::new(10.0, 25.0));
    // SMD USB-C connector (has SMD pads + 4 THT shield tabs)
    let usb = footprint::usb_c_16pin("J1", "USB-C").at_position(Point::new(5.0, 17.5));
    // THT 1x15 Headers
    let h1 = footprint::header_1x15("HDR1", "CONN1").at_position(Point::new(10.0, 3.0));
    let h2 = footprint::header_1x15("HDR2", "CONN2").at_position(Point::new(10.0, 32.0));

    board.add_footprint(mcu);
    board.add_footprint(reg);
    board.add_footprint(usb);
    board.add_footprint(h1);
    board.add_footprint(h2);

    let all_files = export_all_gerbers_and_drill(&board);

    // Verify Drill:
    // USB-C has 4 THT shield tabs (drill 1.0mm)
    // HDR1 has 15 THT pins (drill 1.0mm)
    // HDR2 has 15 THT pins (drill 1.0mm)
    // Total: 4 + 15 + 15 = 34 drill hits
    let drl = parse_drill(all_files.get("drill.drl").unwrap());
    let total_hits: usize = drl.hits_by_tool.values().map(|v| v.len()).sum();
    assert_eq!(total_hits, 34, "Expected exactly 34 drill hits for mixed SMD/THT board");

    // Verify F_Cu contains both SMD pads and THT pads:
    // TQFP-32: 32 SMD pads
    // SOT-223: 4 SMD pads
    // USB-C: 12 SMD pads + 4 THT pads
    // HDR1: 15 THT pads
    // HDR2: 15 THT pads
    // Total F_Cu flashes = 32 + 4 + 12 + 4 + 15 + 15 = 82 flashes
    let fcu = parse_gerber(all_files.get("F_Cu.gbr").unwrap());
    assert_eq!(fcu.flashes.len(), 82, "Expected exactly 82 flashes on F_Cu");

    // Verify B_Cu contains only THT pads (no bottom SMD pads placed):
    // 4 USB tabs + 15 HDR1 + 15 HDR2 = 34 flashes
    let bcu = parse_gerber(all_files.get("B_Cu.gbr").unwrap());
    assert_eq!(bcu.flashes.len(), 34, "Expected exactly 34 flashes on B_Cu (THT pads)");
}

#[test]
fn test_degenerate_zero_length_tracks_and_tiny_apertures() {
    let mut board = PcbBoard::new(50.0, 50.0);

    // Zero-length track (start == end)
    board.add_track(TrackSegment::new(Point::new(20.0, 20.0), Point::new(20.0, 20.0), 0.25, Layer::FCu, 1));

    // Pad with tiny dimension (0.005 mm)
    let tiny_pad = Pad::new_smd("1", "TINY", Point::zero(), PadShape::Rect { width: 0.005, height: 0.005 }, Layer::FCu);
    board.add_footprint(Footprint::new("T1", "VAL", "PKG", Point::new(10.0, 10.0), 0.0, vec![tiny_pad], vec![]));

    let fcu = parse_gerber(&export_gerber_layer(&board, Layer::FCu));
    assert_eq!(fcu.draws.len(), 1);
    assert_eq!(fcu.flashes.len(), 1);

    // Verify aperture has at least 1 nm
    let ap = fcu.apertures.get(&fcu.flashes[0].dcode).unwrap();
    if let ParsedAperture::Rect { w, h } = *ap {
        assert!(w > 0.0 && h > 0.0, "Tiny dimensions must remain positive");
    } else {
        panic!("Expected Rect aperture");
    }
}
