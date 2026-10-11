//! Independent Forensic Auditor Adversarial Stress Test Suite for Milestone 4 (KiCad, Gerber, Drill Exporters)
//!
//! Author: Milestone 4 Forensic Auditor (`m4_auditor`)

use netlist_shatter_viz::pcb::*;

/// Simple S-Expression AST Node for validating KiCad output structure
#[derive(Debug, PartialEq, Clone)]
enum SExpr {
    Atom(String),
    List(Vec<SExpr>),
}

/// Tokenizer for S-expressions
fn tokenize_sexpr(input: &str) -> Vec<String> {
    let mut tokens = Vec::new();
    let mut chars = input.chars().peekable();

    while let Some(&c) = chars.peek() {
        if c.is_whitespace() {
            chars.next();
        } else if c == '(' || c == ')' {
            tokens.push(c.to_string());
            chars.next();
        } else if c == '"' {
            // Quoted string
            chars.next(); // consume opening quote
            let mut s = String::new();
            while let Some(ch) = chars.next() {
                if ch == '\\' {
                    if let Some(escaped) = chars.next() {
                        s.push(escaped);
                    }
                } else if ch == '"' {
                    break;
                } else {
                    s.push(ch);
                }
            }
            tokens.push(format!("\"{}\"", s));
        } else {
            // Unquoted atom
            let mut s = String::new();
            while let Some(&ch) = chars.peek() {
                if ch.is_whitespace() || ch == '(' || ch == ')' || ch == '"' {
                    break;
                }
                s.push(ch);
                chars.next();
            }
            tokens.push(s);
        }
    }
    tokens
}

/// Recursive descent parser for S-expressions
fn parse_sexpr_tokens(tokens: &[String], pos: &mut usize) -> Result<SExpr, String> {
    if *pos >= tokens.len() {
        return Err("Unexpected EOF".to_string());
    }

    let token = &tokens[*pos];
    *pos += 1;

    if token == "(" {
        let mut list = Vec::new();
        while *pos < tokens.len() && tokens[*pos] != ")" {
            let child = parse_sexpr_tokens(tokens, pos)?;
            list.push(child);
        }
        if *pos >= tokens.len() || tokens[*pos] != ")" {
            return Err("Mismatched opening parenthesis".to_string());
        }
        *pos += 1; // consume ")"
        Ok(SExpr::List(list))
    } else if token == ")" {
        Err("Unexpected closing parenthesis".to_string())
    } else {
        Ok(SExpr::Atom(token.clone()))
    }
}

fn parse_sexpr(s: &str) -> Result<SExpr, String> {
    let tokens = tokenize_sexpr(s);
    let mut pos = 0;
    let ast = parse_sexpr_tokens(&tokens, &mut pos)?;
    if pos != tokens.len() {
        return Err(format!("Trailing tokens at pos {}/{}", pos, tokens.len()));
    }
    Ok(ast)
}

// ============================================================================
// Adversarial Test 1: KiCad S-Expression AST Validation Under High Density
// ============================================================================

#[test]
fn test_adversarial_kicad_ast_validation_high_density() {
    let mut board = PcbBoard::new(120.0, 85.0);

    // Register 20 distinct nets with special characters and symbols
    for i in 1..=20 {
        board.register_net(i, &format!("NET_{}_+5V/CLK_\"SPECIAL\"\\{}", i, i * 3));
    }

    // Add 40 mixed footprints with various rotations and positions
    for i in 0..40 {
        let x = 10.0 + (i % 8) as f64 * 12.0;
        let y = 10.0 + (i / 8) as f64 * 14.0;
        let rot = (i as f64 * 45.0) % 360.0;

        if i % 3 == 0 {
            let r = resistor_0805(&format!("R{}", i), "10k")
                .at_position(Point::new(x, y))
                .with_rotation(rot)
                .with_net_assigned("1", (i % 20 + 1) as u32)
                .with_net_assigned("2", ((i + 1) % 20 + 1) as u32);
            board.add_footprint(r);
        } else if i % 3 == 1 {
            let j = header_1x15(&format!("J{}", i), "CONN_1x15")
                .at_position(Point::new(x, y))
                .with_rotation(rot)
                .with_net_assigned("1", 1);
            board.add_footprint(j);
        } else {
            let c = capacitor_0805(&format!("C{}", i), "100nF")
                .at_position(Point::new(x, y))
                .with_rotation(rot)
                .with_net_assigned("1", 1)
                .with_net_assigned("2", 2);
            board.add_footprint(c);
        }
    }

    // Add 50 tracks across both layers
    for i in 0..50 {
        let layer = if i % 2 == 0 { Layer::FCu } else { Layer::BCu };
        board.add_track(TrackSegment::new(
            Point::new(10.0 + i as f64, 15.0),
            Point::new(10.0 + i as f64, 35.0),
            0.20 + (i as f64 * 0.01),
            layer,
            (i % 20 + 1) as u32,
        ));
    }

    // Add 25 vias
    for i in 0..25 {
        board.add_via(Via::new(
            Point::new(15.0 + i as f64 * 3.0, 25.0),
            0.8,
            0.4,
            (i % 20 + 1) as u32,
        ));
    }

    let output = export_kicad_pcb(&board);

    // 1. Structural parser must succeed without errors
    let ast = parse_sexpr(&output).expect("KiCad output must be valid S-expression grammar");

    // 2. Validate Root element
    match ast {
        SExpr::List(elements) => {
            assert!(!elements.is_empty());
            assert_eq!(elements[0], SExpr::Atom("kicad_pcb".to_string()));

            let mut footprint_count = 0;
            let mut segment_count = 0;
            let mut via_count = 0;
            let mut edge_cuts_count = 0;

            for elem in &elements[1..] {
                if let SExpr::List(sub) = elem {
                    if !sub.is_empty() {
                        if let SExpr::Atom(ref tag) = sub[0] {
                            match tag.as_str() {
                                "footprint" => footprint_count += 1,
                                "segment" => segment_count += 1,
                                "via" => via_count += 1,
                                "gr_line" => edge_cuts_count += 1,
                                _ => {}
                            }
                        }
                    }
                }
            }

            assert_eq!(footprint_count, 40, "Must contain all 40 footprints");
            assert_eq!(segment_count, 50, "Must contain all 50 tracks");
            assert_eq!(via_count, 25, "Must contain all 25 vias");
            assert!(edge_cuts_count >= 4, "Must contain at least 4 edge cuts lines");
        }
        _ => panic!("Expected root SExpr::List"),
    }
}

// ============================================================================
// Adversarial Test 2: RS-274X Gerber Aperture Synthesis & Coordinate Precision
// ============================================================================

#[test]
fn test_adversarial_gerber_aperture_synthesis_and_deduplication() {
    let mut board = PcbBoard::new(70.0, 45.0);

    // Add 10 identical 0805 resistors (each has two 1.0mm x 1.3mm pads)
    for i in 0..10 {
        let r = resistor_0805(&format!("R{}", i), "10k")
            .at_position(Point::new(10.0 + i as f64 * 5.0, 20.0))
            .with_rotation(0.0);
        board.add_footprint(r);
    }

    // Add 15 tracks with identical width (0.25mm)
    for i in 0..15 {
        board.add_track(TrackSegment::new(
            Point::new(5.0, 5.0 + i as f64),
            Point::new(20.0, 5.0 + i as f64),
            0.25,
            Layer::FCu,
            1,
        ));
    }

    // Add 10 vias with identical diameter (0.80mm)
    for i in 0..10 {
        board.add_via(Via::new(
            Point::new(25.0, 5.0 + i as f64 * 2.0),
            0.80,
            0.40,
            1,
        ));
    }

    let fcu_gbr = export_gerber_layer(&board, Layer::FCu);

    // Count aperture definitions %ADDnn...*%
    let mut add_directives = Vec::new();
    for line in fcu_gbr.lines() {
        if line.starts_with("%ADD") {
            add_directives.push(line);
        }
    }

    // Resistors have 1 unique pad aperture (1.0x1.3mm rectangle)
    // Tracks have 1 unique circle aperture (0.25mm)
    // Vias have 1 unique circle aperture (0.80mm)
    // TOTAL unique apertures must be exactly 3!
    assert_eq!(
        add_directives.len(),
        3,
        "Deduplication must result in exactly 3 unique apertures, found {}: {:?}",
        add_directives.len(),
        add_directives
    );

    // Verify D-codes start at 10 and are sequential: D10, D11, D12
    assert!(add_directives.iter().any(|s| s.starts_with("%ADD10")));
    assert!(add_directives.iter().any(|s| s.starts_with("%ADD11")));
    assert!(add_directives.iter().any(|s| s.starts_with("%ADD12")));

    // Verify coordinates in draws and flashes have 10^6 integer scale
    // Point (5.0, 5.0) -> X5000000Y40000000
    assert!(
        fcu_gbr.contains("X5000000Y40000000"),
        "Coordinates must be exactly scaled by 10^6"
    );
    // Point (20.0, 5.0) -> X20000000Y40000000
    assert!(
        fcu_gbr.contains("X20000000Y40000000"),
        "Coordinates must be exactly scaled by 10^6"
    );
}

// ============================================================================
// Adversarial Test 3: Gerber Pad Rotation Aperture Transposition (90° / 270°)
// ============================================================================

#[test]
fn test_adversarial_gerber_pad_rotation_transposition() {
    let mut board = PcbBoard::new(50.0, 50.0);

    // Resistor R1 unrotated: 0805 pads are Rect { width: 1.0, height: 1.3 }
    let r1 = resistor_0805("R1", "10k")
        .at_position(Point::new(15.0, 15.0))
        .with_rotation(0.0);
    board.add_footprint(r1);

    // Resistor R2 rotated 90 deg: aperture width and height must swap to 1.3 x 1.0
    let r2 = resistor_0805("R2", "10k")
        .at_position(Point::new(30.0, 30.0))
        .with_rotation(90.0);
    board.add_footprint(r2);

    let gbr = export_gerber_layer(&board, Layer::FCu);

    // Both apertures (1.0000X1.3000 and 1.3000X1.0000) must be defined
    assert!(
        gbr.contains("1.0000X1.3000") || gbr.contains("1.000X1.300"),
        "Must contain unrotated aperture 1.0000X1.3000"
    );
    assert!(
        gbr.contains("1.3000X1.0000") || gbr.contains("1.300X1.000"),
        "Must contain transposed aperture 1.3000X1.0000 for 90-degree rotated pad"
    );
}

// ============================================================================
// Adversarial Test 4: Excellon Drill Ascending Sorting & Hit Coordinate Matching
// ============================================================================

#[test]
fn test_adversarial_excellon_drill_sorting_and_exact_coordinates() {
    let mut board = PcbBoard::new(80.0, 60.0);

    // Add vias with 3 different drill sizes in arbitrary order
    board.add_via(Via::new(Point::new(30.0, 40.0), 1.2, 0.6, 1));
    board.add_via(Via::new(Point::new(10.0, 20.0), 0.6, 0.3, 1));
    board.add_via(Via::new(Point::new(20.0, 30.0), 0.8, 0.4, 1));

    // Add THT header with 1.0mm drill
    let j1 = header_1x15("J1", "CONN_1x15")
        .at_position(Point::new(15.0, 10.0))
        .with_rotation(0.0);
    board.add_footprint(j1); // 15 holes, 1.0mm drill

    let drl = export_excellon_drill(&board);

    // 1. Tool Table Ordering Verification
    let mut tool_definitions = Vec::new();
    for line in drl.lines() {
        if line.starts_with('T') && line.contains('C') {
            tool_definitions.push(line.to_string());
        }
    }

    assert_eq!(
        tool_definitions.len(),
        4,
        "Must define exactly 4 drill tools (0.3mm, 0.4mm, 0.6mm, 1.0mm)"
    );

    // Must be in ascending diameter order:
    // T01C0.300
    // T02C0.400
    // T03C0.600
    // T04C1.000
    assert!(tool_definitions[0].contains("T01C0.300"));
    assert!(tool_definitions[1].contains("T02C0.400"));
    assert!(tool_definitions[2].contains("T03C0.600"));
    assert!(tool_definitions[3].contains("T04C1.000"));

    // 2. Total Hit Count
    // 3 vias + 15 header pins = 18 hits
    let hit_lines: Vec<&str> = drl
        .lines()
        .filter(|line| line.starts_with('X') && line.contains('Y'))
        .collect();
    assert_eq!(hit_lines.len(), 18, "Expected exactly 18 drill hit coordinates");

    // 3. Exact Coordinate Presence
    assert!(drl.contains("X10.000Y40.000"), "Must contain hit for via 1");
    assert!(drl.contains("X20.000Y30.000"), "Must contain hit for via 2");
    assert!(drl.contains("X30.000Y20.000"), "Must contain hit for via 3");

    // 4. Hit Sorting Under T04 (Header 1x15):
    // Header J1 placed at (15.0, 10.0), with pins along Y from 10.0 to 10.0 + 14 * 2.54 = 45.56
    // Coordinates under T04 must be strictly non-decreasing in Y
    let lines: Vec<&str> = drl.lines().collect();
    let t04_pos = lines
        .iter()
        .position(|&l| l == "T04")
        .expect("T04 tool selection must be present");
    let mut prev_y = -1.0;
    for &line in &lines[t04_pos + 1..] {
        if line.starts_with('T') || line.starts_with('M') {
            break;
        }
        if line.starts_with('X') && line.contains('Y') {
            let y_part = &line[line.find('Y').unwrap() + 1..];
            let y_val: f64 = y_part.parse().expect("Valid float coordinate");
            assert!(
                y_val >= prev_y - 1e-4,
                "Drill hits under tool must be sorted by Y ascending: {} < {}",
                y_val,
                prev_y
            );
            prev_y = y_val;
        }
    }
}

// ============================================================================
// Adversarial Test 5: Dynamic Responsiveness & Anti-Facade Probe
// ============================================================================

#[test]
fn test_adversarial_dynamic_procedural_anti_facade_probe() {
    // Generate 3 completely different boards procedurally
    for run in 1..=3 {
        let board_w = 40.0 + run as f64 * 25.0;
        let board_h = 30.0 + run as f64 * 20.0;
        let mut board = PcbBoard::new(board_w, board_h);

        let net_name = format!("PROCEDURAL_NET_{}", run * 101);
        board.register_net(run as u32, &net_name);

        let trk_start = Point::new(run as f64 * 3.14, run as f64 * 2.71);
        let trk_end = Point::new(run as f64 * 7.89, run as f64 * 9.12);
        let trk_width = 0.15 + (run as f64 * 0.07);
        board.add_track(TrackSegment::new(trk_start, trk_end, trk_width, Layer::FCu, run as u32));

        // 1. Verify KiCad output reflects exact procedural values
        let kicad = export_kicad_pcb(&board);
        assert!(
            kicad.contains(&format!("(net {} \"{}\")", run, net_name)),
            "KiCad export must contain procedural net"
        );
        let expected_x = format!("{:.4}", (trk_start.x * 10000.0).round() / 10000.0);
        let expected_x_trimmed = expected_x.trim_end_matches('0').trim_end_matches('.');
        assert!(
            kicad.contains(expected_x_trimmed),
            "KiCad export must dynamically reflect track start X: {}",
            expected_x_trimmed
        );

        // 2. Verify Gerber output reflects exact procedural coordinates
        let gbr = export_gerber_layer(&board, Layer::FCu);
        let scaled_x = (trk_start.x * 1_000_000.0).round() as i64;
        let scaled_y = ((board.height - trk_start.y) * 1_000_000.0).round() as i64;
        assert!(
            gbr.contains(&format!("X{}Y{}", scaled_x, scaled_y)),
            "Gerber export must dynamically reflect procedural coordinates X{}Y{}",
            scaled_x,
            scaled_y
        );

        // 3. Verify Edge.Cuts bounds
        let edge_gbr = export_gerber_layer(&board, Layer::EdgeCuts);
        let scaled_w = (board_w * 1_000_000.0).round() as i64;
        let scaled_h = (board_h * 1_000_000.0).round() as i64;
        assert!(
            edge_gbr.contains(&format!("X{}Y{}", scaled_w, scaled_h)),
            "Edge cuts must reflect board width and height X{}Y{}",
            scaled_w,
            scaled_h
        );
    }
}

// ============================================================================
// Adversarial Test 6: Multi-File Fabrication Package Integrity
// ============================================================================

#[test]
fn test_adversarial_all_gerbers_and_drill_package_completeness() {
    let mut board = PcbBoard::new(65.0, 45.0);
    board.register_net(1, "GND");
    board.register_net(2, "VCC");

    let r = resistor_0805("R1", "10k").at_position(Point::new(20.0, 20.0));
    board.add_footprint(r);

    let j = header_1x15("J1", "CONN_1x15").at_position(Point::new(10.0, 5.0));
    board.add_footprint(j);

    board.add_track(TrackSegment::new(Point::new(20.0, 20.0), Point::new(30.0, 20.0), 0.25, Layer::FCu, 1));
    board.add_track(TrackSegment::new(Point::new(30.0, 20.0), Point::new(30.0, 30.0), 0.25, Layer::BCu, 2));
    board.add_via(Via::new(Point::new(30.0, 20.0), 0.8, 0.4, 1));

    let pkg = export_all_gerbers_and_drill(&board);

    // Verify all 7 files exist
    let required_files = [
        "F_Cu.gbr",
        "B_Cu.gbr",
        "F_SilkS.gbr",
        "Edge_Cuts.gbr",
        "F_Mask.gbr",
        "B_Mask.gbr",
        "drill.drl",
    ];

    for fname in &required_files {
        assert!(pkg.contains_key(*fname), "Package missing required file: {}", fname);
        let content = &pkg[*fname];
        assert!(!content.is_empty(), "File {} must not be empty", fname);

        if fname.ends_with(".gbr") {
            assert!(content.contains("%FSLAX46Y46*%"), "{} missing %FSLAX46Y46*%", fname);
            assert!(content.contains("%MOMM*%"), "{} missing %MOMM*%", fname);
            assert!(content.trim_end().ends_with("M02*"), "{} missing M02*", fname);
        } else if fname.ends_with(".drl") {
            assert!(content.starts_with("M48"), "drill.drl missing M48");
            assert!(content.trim_end().ends_with("M30"), "drill.drl missing M30");
        }
    }
}
