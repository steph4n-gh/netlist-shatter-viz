//! Comprehensive Verification & Test Suite for the Universal Specctra Engine
//!
//! Verifies:
//! 1. S-expression lexer & AST parser edge cases (quotes, escapes, comments, malformed syntax)
//! 2. Engineering unit conversion across `um`, `mm`, `mil`, `inch`, `cm` and arbitrary resolutions
//! 3. KiCad Specctra DSN dialect ingestion (inverted Y, um units, polyline boundaries, padstack parsing)
//! 4. Altium Specctra DSN dialect ingestion (mil units, TopLayer/BottomLayer, rect boundaries)
//! 5. Eagle Specctra DSN dialect ingestion (mm units, multiple pin formats)
//! 6. SMD pad shapes (Rect, Circle, Obround) and Through-Hole pads with drills
//! 7. Multi-pin nets (3+ pins) requiring Steiner-like minimum spanning tree routing
//! 8. Congestion-negotiated Rip-Up & Reroute (RRR) on crossing nets with automated via insertion
//! 9. Keepout obstacle avoidance
//! 10. Specctra Session (`.ses`) exporter standards compliance verified for KiCad import
//! 11. Zero DRC clearance violations, shorts, or unrouted nets on all routed boards
//! 12. Bidirectional roundtrip: `PcbBoard -> DSN -> Route -> SES`

use netlist_shatter_viz::pcb::*;
use std::fs;

// ============================================================================
// 1. S-Expression Tokenizer & AST Tests
// ============================================================================

#[test]
fn test_sexpr_parser_deeply_nested_lists() {
    let input = "(a (b (c (d (e 42)))))";
    let ast = parse_sexpr(input).expect("parse deeply nested");
    assert_eq!(ast.len(), 1);

    let mut current = &ast[0];
    for expected_name in &["a", "b", "c", "d", "e"] {
        assert_eq!(current.name(), Some(*expected_name));
        if *expected_name != "e" {
            current = current.nth(1).expect("nested child");
        }
    }
    assert_eq!(current.nth_f64(1), Some(42.0));
}

#[test]
fn test_sexpr_parser_malformed_unbalanced_parentheses() {
    let unclosed = "(pcb \"unclosed\" (structure (layer F.Cu))";
    assert!(parse_sexpr(unclosed).is_err());

    let extra_close = "(pcb \"test\"))";
    assert!(parse_sexpr(extra_close).is_err());
}

#[test]
fn test_sexpr_quoted_strings_and_escapes() {
    let input = r#"
        ; Specctra comment
        (pcb "Design with \"quotes\" and spaces"
            # Hash comment
            (host_cad 'Altium "Designer" Pro')
            (constant "A""B" 123)
        )
    "#;
    let ast = parse_sexpr(input).expect("parse quotes and escapes");
    let pcb = &ast[0];
    assert_eq!(pcb.nth_atom(1), Some("Design with \"quotes\" and spaces"));

    let host = pcb.child("host_cad").expect("host_cad");
    assert_eq!(host.nth_atom(1), Some("Altium \"Designer\" Pro"));

    let constant = pcb.child("constant").expect("constant");
    assert_eq!(constant.nth_atom(1), Some("A\"B"));
    assert_eq!(constant.nth_atom(2), Some("123"));
}

#[test]
fn test_sexpr_empty_and_whitespace_only() {
    assert!(parse_sexpr("").unwrap().is_empty());
    assert!(parse_sexpr("   \n\t   ; comment only \n").unwrap().is_empty());
}

// ============================================================================
// 2. Unit and Resolution Conversion Tests
// ============================================================================

#[test]
fn test_unit_resolution_metrics() {
    // 1 mm in mm (factor 1.0)
    let res_mm = SpecctraResolution::new(SpecctraUnit::Mm, 1.0);
    assert_eq!(res_mm.to_mm(10.5), 10.5);
    assert_eq!(res_mm.from_mm(10.5), 10.5);

    // 1000 coordinate units = 1 mm (e.g. integer micron coordinates)
    let res_mm1000 = SpecctraResolution::new(SpecctraUnit::Mm, 1000.0);
    assert_eq!(res_mm1000.to_mm(25.4), 25.4);
    assert_eq!(res_mm1000.from_mm(25.4), 25.4);

    // (resolution um 10): in KiCad coordinates are in micrometers (1000 um = 1 mm)
    let res_kicad = SpecctraResolution::new(SpecctraUnit::Um, 10.0);
    assert_eq!(res_kicad.to_mm(1000.0), 1.0);
    assert_eq!(res_kicad.from_mm(1.0), 1000.0);

    // (resolution mil 10): in Altium coordinates are in mils (100 mil = 2.54 mm)
    let res_mil = SpecctraResolution::new(SpecctraUnit::Mil, 10.0);
    assert!((res_mil.to_mm(100.0) - 2.54).abs() < 1e-9);
    assert!((res_mil.from_mm(2.54) - 100.0).abs() < 1e-9);

    // 10000 coordinate units = 1 inch = 25.4 mm
    let res_inch = SpecctraResolution::new(SpecctraUnit::Inch, 10000.0);
    assert!((res_inch.to_mm(1.0) - 25.4).abs() < 1e-9);
    assert!((res_inch.from_mm(25.4) - 1.0).abs() < 1e-9);
}

// ============================================================================
// 3. KiCad DSN Dialect Ingestion & Routing
// ============================================================================

const KICAD_DSN_SAMPLE: &str = r#"
(pcb "kicad_test_board.dsn"
  (parser
    (string_quote ")
    (space_in_quoted_tokens on)
    (host_cad "KiCad's Pcbnew")
    (host_version "7.0.0")
  )
  (resolution um 10)
  (unit um)
  (structure
    (layer "F.Cu" (type signal) (property (index 0)))
    (layer "B.Cu" (type signal) (property (index 1)))
    (boundary
      (path pcb 0 0 0 50000 0 50000 -40000 0 -40000 0 0)
    )
    (via "Via[0-1]_800:400_um")
    (rule
      (width 250)
      (clearance 200)
    )
  )
  (placement
    (component "Resistor_SMD:R_0805_2012Metric"
      (place "R1" 10000 -20000 front 0 (pn "10k"))
      (place "R2" 35000 -20000 front 0 (pn "10k"))
    )
  )
  (library
    (image "Resistor_SMD:R_0805_2012Metric"
      (pin "Rect:1000x1000_um" "1" -1000 0)
      (pin "Rect:1000x1000_um" "2" 1000 0)
    )
    (padstack "Rect:1000x1000_um"
      (shape (rect "F.Cu" 1000 1000))
    )
    (padstack "Via[0-1]_800:400_um"
      (shape (circle "F.Cu" 800))
      (shape (circle "B.Cu" 800))
    )
  )
  (network
    (net "NET_SIG"
      (pins "R1"-"2" "R2"-"1")
      (rule (width 250) (clearance 200))
    )
  )
)
"#;

#[test]
fn test_kicad_dsn_parsing_and_ingestion() {
    let dsn = parse_specctra_dsn(KICAD_DSN_SAMPLE).expect("parse kicad dsn");
    assert_eq!(dsn.pcb_name, "kicad_test_board.dsn");
    assert_eq!(dsn.resolution.unit, SpecctraUnit::Um);
    assert_eq!(dsn.resolution.factor, 10.0);

    // Board boundary: [0, 50000] x [-40000, 0] um = 50.0 mm x 40.0 mm
    assert!((dsn.bounds.width() - 50.0).abs() < 1e-3);
    assert!((dsn.bounds.height() - 40.0).abs() < 1e-3);
    // Verify layers
    assert_eq!(dsn.layers.len(), 2);
    assert_eq!(dsn.layers[0].name, "F.Cu");
    assert_eq!(dsn.layers[1].name, "B.Cu");

    // Ingest into PcbBoard
    let (board, config) = dsn.to_board().expect("ingest into board");
    assert_eq!(board.footprints.len(), 2);
    assert_eq!(config.default_trace_width, 0.25);
    assert_eq!(config.clearance, 0.20);
    assert_eq!(config.via_diameter, 0.80);
    assert_eq!(config.via_drill, 0.40);

    // Verify pin net assignment
    let r1 = board.find_footprint("R1").expect("R1 found");
    assert_eq!(r1.pad("2").unwrap().net, Some(1));
    let r2 = board.find_footprint("R2").expect("R2 found");
    assert_eq!(r2.pad("1").unwrap().net, Some(1));
}

#[test]
fn test_kicad_dsn_routing_and_ses_generation() {
    let (ses, report) = route_dsn_to_ses(KICAD_DSN_SAMPLE, None).expect("route kicad dsn");
    assert_eq!(report.routed_nets, 1);
    assert_eq!(report.unrouted_nets, 0);
    assert!(report.total_tracks > 0);

    // Check SES content structure
    assert!(ses.contains("(session \"kicad_test_board.dsn\""));
    assert!(ses.contains("(base_design \"kicad_test_board.dsn\")"));
    assert!(ses.contains("(routes"));
    assert!(ses.contains("(resolution um"));
    assert!(ses.contains("(library_out"));
    assert!(ses.contains("(network_out"));
    assert!(ses.contains("(net \"NET_SIG\""));
    assert!(ses.contains("(wire (path \"F.Cu\""));

    // Verify parentheses balance
    let open_count = ses.chars().filter(|&c| c == '(').count();
    let close_count = ses.chars().filter(|&c| c == ')').count();
    assert_eq!(open_count, close_count, "SES parentheses must be strictly balanced");
}

// ============================================================================
// 4. Altium DSN Dialect Ingestion & Routing
// ============================================================================

const ALTIUM_DSN_SAMPLE: &str = r#"
(pcb AltiumBoard
  (parser
    (string_quote ")
    (space_in_quoted_tokens on)
    (host_cad "Altium Designer")
  )
  (unit mil)
  (resolution mil 10)
  (structure
    (layer TopLayer (type signal))
    (layer BottomLayer (type signal))
    (boundary (rect pcb 0 0 1500 1200))
    (via Via_24_12_mil)
    (rule (width 10) (clearance 8))
  )
  (placement
    (component CAP_0603
      (place C1 300 600 front 0)
      (place C2 1000 600 front 0)
    )
  )
  (library
    (image CAP_0603
      (pin Pad_30x30_mil 1 -25 0)
      (pin Pad_30x30_mil 2 25 0)
    )
    (padstack Pad_30x30_mil
      (shape (rect TopLayer -15 -15 15 15))
    )
    (padstack Via_24_12_mil
      (shape (circle TopLayer 24))
      (shape (circle BottomLayer 24))
    )
  )
  (network
    (net "BUS_PWR" (pins C1-1 C2-1))
    (net "BUS_GND" (pins C1-2 C2-2))
  )
)
"#;

#[test]
fn test_altium_dsn_routing_and_drc() {
    let dsn = parse_specctra_dsn(ALTIUM_DSN_SAMPLE).expect("parse altium dsn");
    assert_eq!(dsn.resolution.unit, SpecctraUnit::Mil);
    assert_eq!(dsn.resolution.factor, 10.0);
    assert_eq!(dsn.layers.len(), 2);
    assert_eq!(dsn.layers[0].name, "TopLayer");
    assert_eq!(dsn.layers[1].name, "BottomLayer");

    let (mut board, config) = dsn.to_board().expect("ingest altium board");
    assert_eq!(board.footprints.len(), 2);

    let report = route_board_with_report(&mut board, &config).expect("route altium board");
    assert_eq!(report.routed_nets, 2);
    assert_eq!(report.unrouted_nets, 0);

    // Verify DRC: 0 clearance violations, 0 shorts, 0 unrouted nets
    let drc = run_drc(&board, &config);
    assert_eq!(drc.unrouted_nets.len(), 0, "All nets must be routed");
    assert_eq!(drc.clearance_violations.len(), 0, "No clearance violations");
    assert_eq!(drc.short_circuits.len(), 0, "No short circuits");

    // Export SES and verify layer names are preserved as "TopLayer" / "BottomLayer"
    let ses = export_specctra_ses(&board, &dsn);
    assert!(ses.contains("TopLayer"));
}

// ============================================================================
// 5. Eagle DSN Dialect Ingestion & Multi-Pin Routing
// ============================================================================

const EAGLE_DSN_SAMPLE: &str = r#"
(pcb "eagle_design"
  (parser
    (string_quote ")
    (space_in_quoted_tokens on)
    (host_cad "EAGLE")
  )
  (unit um)
  (resolution um 1000)
  (structure
    (layer Top (type signal))
    (layer Bottom (type signal))
    (boundary (path pcb 0.1 0 0 60000 0 60000 50000 0 50000 0 0))
    (via "Via:0.8_0.4mm")
    (rule (width 250) (clearance 200))
  )
  (placement
    (component "SOIC-8"
      (place U1 15000 25000 front 0)
    )
    (component "LED"
      (place D1 35000 15000 front 0)
      (place D2 35000 35000 front 0)
    )
  )
  (library
    (image "SOIC-8"
      (pin "SMD_1.2x0.6" 1 -2000 -1500)
      (pin "SMD_1.2x0.6" 2 -2000 1500)
    )
    (image "LED"
      (pin "SMD_1.0x1.0" 1 -1000 0)
      (pin "SMD_1.0x1.0" 2 1000 0)
    )
    (padstack "SMD_1.2x0.6"
      (shape (rect Top 1200 600))
    )
    (padstack "SMD_1.0x1.0"
      (shape (circle Top 1000))
    )
    (padstack "Via:0.8_0.4mm"
      (shape (circle Top 800))
      (shape (circle Bottom 800))
    )
  )
  (network
    ; 3-pin net: U1-1 connects to D1-1 and D2-1
    (net "BUS_3PIN" (pins U1-1 D1-1 D2-1))
    ; 2-pin net: U1-2 connects to D1-2
    (net "SIG_RETURN" (pins U1-2 D1-2))
  )
)
"#;

#[test]
fn test_eagle_multi_pin_net_routing_and_drc() {
    let dsn = parse_specctra_dsn(EAGLE_DSN_SAMPLE).expect("parse eagle dsn");
    assert_eq!(dsn.resolution.unit, SpecctraUnit::Um);
    assert_eq!(dsn.resolution.factor, 1000.0);

    let (mut board, config) = dsn.to_board().expect("ingest eagle board");
    assert_eq!(board.footprints.len(), 3);

    // Route multi-pin nets
    let report = route_board_with_report(&mut board, &config).expect("route eagle board");
    assert_eq!(report.routed_nets, 2);
    assert_eq!(report.unrouted_nets, 0);

    // Verify DRC passes completely
    let drc = run_drc(&board, &config);
    assert_eq!(drc.unrouted_nets.len(), 0);
    assert_eq!(drc.clearance_violations.len(), 0);
    assert_eq!(drc.short_circuits.len(), 0);

    // Export SES and verify format
    let ses = export_specctra_ses(&board, &dsn);
    assert!(ses.contains("(net \"BUS_3PIN\""));
    assert!(ses.contains("(net \"SIG_RETURN\""));
}

// ============================================================================
// 6. Crossing Nets: Rip-Up & Reroute with Automated Via Insertion
// ============================================================================

#[test]
fn test_crossing_nets_rrr_via_insertion() {
    // Two nets crossing each other in a cross (+) configuration:
    // Net 1: horizontal (10, 25) -> (40, 25)
    // Net 2: vertical   (25, 10) -> (25, 40)
    let dsn_content = r#"
    (pcb "crossing_test"
      (resolution um 1000)
      (unit um)
      (structure
        (layer F.Cu (type signal))
        (layer B.Cu (type signal))
        (boundary (rect pcb 0 0 50000 50000))
        (via "Via_800:400")
        (rule (width 250) (clearance 200))
      )
      (placement
        (component PAD_COMP
          (place U1 10000 25000 front 0)
          (place U2 40000 25000 front 0)
          (place U3 25000 10000 front 0)
          (place U4 25000 40000 front 0)
        )
      )
      (library
        (image PAD_COMP
          (pin P1 1 0 0)
        )
        (padstack P1
          (shape (rect F.Cu 1000 1000))
        )
        (padstack "Via_800:400"
          (shape (circle F.Cu 800))
          (shape (circle B.Cu 800))
        )
      )
      (network
        (net "NET_H" (pins U1-1 U2-1))
        (net "NET_V" (pins U3-1 U4-1))
      )
    )
    "#;

    let dsn = parse_specctra_dsn(dsn_content).expect("parse crossing dsn");
    let (mut board, config) = dsn.to_board().expect("ingest crossing board");

    let report = route_board_with_report(&mut board, &config).expect("route crossing nets");
    assert_eq!(report.routed_nets, 2);
    assert_eq!(report.unrouted_nets, 0);

    // Because the nets cross, at least one via must have been inserted
    assert!(
        report.total_vias > 0 || board.vias.len() > 0,
        "Crossing nets must use vias to switch layers and prevent short circuits"
    );

    // Verify 0 DRC violations
    let drc = run_drc(&board, &config);
    assert_eq!(drc.unrouted_nets.len(), 0);
    assert_eq!(drc.clearance_violations.len(), 0);
    assert_eq!(drc.short_circuits.len(), 0);

    // Export SES
    let ses = export_specctra_ses(&board, &dsn);
    assert!(ses.contains("(via \"Via[0-1]_"));
}

// ============================================================================
// 7. Keepout Obstacle Avoidance Test
// ============================================================================

#[test]
fn test_keepout_obstacle_avoidance() {
    // Two pads at (10, 20) and (40, 20).
    // A keepout block is placed right in the middle at x=[20, 30], y=[15, 25].
    // The router must pathfind around it.
    let dsn_content = r#"
    (pcb "keepout_test"
      (resolution um 1000)
      (unit um)
      (structure
        (layer F.Cu (type signal))
        (layer B.Cu (type signal))
        (boundary (rect pcb 0 0 50000 40000))
        (keepout KO_BLOCK
          (rect pcb 20000 15000 30000 25000)
        )
        (rule (width 250) (clearance 200))
      )
      (placement
        (component TEST_PAD
          (place P1 10000 20000 front 0)
          (place P2 40000 20000 front 0)
        )
      )
      (library
        (image TEST_PAD
          (pin P_PIN 1 0 0)
        )
        (padstack P_PIN
          (shape (circle F.Cu 1000))
        )
      )
      (network
        (net "AROUND_KO" (pins P1-1 P2-1))
      )
    )
    "#;

    let dsn = parse_specctra_dsn(dsn_content).expect("parse keepout dsn");
    assert_eq!(dsn.keepouts.len(), 1);

    let (mut board, config) = dsn.to_board().expect("ingest keepout board");
    let report = route_board_with_report(&mut board, &config).expect("route around keepout");
    assert_eq!(report.routed_nets, 1);

    // Verify none of the track points penetrate the keepout interior [20, 30] x [15, 25]
    for seg in &board.tracks {
        let mid = seg.midpoint();
        let inside_ko = mid.x > 20.0 && mid.x < 30.0 && mid.y > 15.0 && mid.y < 25.0;
        assert!(!inside_ko, "Track segment midpoint ({:.2},{:.2}) penetrated keepout!", mid.x, mid.y);
    }

    let drc = run_drc(&board, &config);
    assert_eq!(drc.unrouted_nets.len(), 0);
    assert_eq!(drc.clearance_violations.len(), 0);
    assert_eq!(drc.short_circuits.len(), 0);
}

// ============================================================================
// 8. Bidirectional Roundtrip: PcbBoard -> DSN -> Route -> SES
// ============================================================================

#[test]
fn test_bidirectional_roundtrip() {
    // 1. Create a synthetic PcbBoard
    let mut board = PcbBoard::new(60.0, 40.0);
    let net1 = board.get_or_create_net("NET_SIG1");
    let net2 = board.get_or_create_net("NET_SIG2");

    let pad1 = Pad::new_smd("1", "1", Point::zero(), PadShape::Rect { width: 1.2, height: 1.2 }, Layer::FCu).with_net(net1);
    let pad2 = Pad::new_smd("2", "2", Point::new(2.54, 0.0), PadShape::Rect { width: 1.2, height: 1.2 }, Layer::FCu).with_net(net2);
    let fp1 = Footprint::new("U1", "SOIC", "SOIC", Point::new(15.0, 20.0), 0.0, vec![pad1, pad2], Vec::new());

    let pad3 = Pad::new_smd("1", "1", Point::zero(), PadShape::Rect { width: 1.2, height: 1.2 }, Layer::FCu).with_net(net1);
    let pad4 = Pad::new_smd("2", "2", Point::new(2.54, 0.0), PadShape::Rect { width: 1.2, height: 1.2 }, Layer::FCu).with_net(net2);
    let fp2 = Footprint::new("U2", "SOIC", "SOIC", Point::new(45.0, 20.0), 0.0, vec![pad3, pad4], Vec::new());

    board.add_footprint(fp1);
    board.add_footprint(fp2);

    // 2. Export board as Specctra DSN
    let dsn_config = SpecctraDsnExportConfig {
        pcb_name: "roundtrip_board".to_string(),
        unit: SpecctraUnit::Mm,
        resolution_factor: 1000.0,
        top_layer_name: "F.Cu".to_string(),
        bot_layer_name: "B.Cu".to_string(),
    };
    let dsn_text = export_specctra_dsn(&board, &dsn_config);

    assert!(dsn_text.contains("(pcb \"roundtrip_board\""));
    assert!(dsn_text.contains("(net \"NET_SIG1\""));
    assert!(dsn_text.contains("(net \"NET_SIG2\""));

    // 3. Parse DSN text back into SpecctraDsn
    let parsed_dsn = parse_specctra_dsn(&dsn_text).expect("parse exported dsn");
    assert_eq!(parsed_dsn.pcb_name, "roundtrip_board");

    // 4. Ingest parsed DSN into a new board and route it
    let (mut routed_board, config) = parsed_dsn.to_board().expect("ingest exported dsn");
    let report = route_board_with_report(&mut routed_board, &config).expect("route parsed board");
    assert_eq!(report.routed_nets, 2);
    assert_eq!(report.unrouted_nets, 0);

    // 5. Verify 0 DRC violations
    let drc = run_drc(&routed_board, &config);
    assert_eq!(drc.unrouted_nets.len(), 0);
    assert_eq!(drc.clearance_violations.len(), 0);
    assert_eq!(drc.short_circuits.len(), 0);

    // 6. Export SES and verify format
    let ses = export_specctra_ses(&routed_board, &parsed_dsn);
    assert!(ses.contains("(session \"roundtrip_board\""));
    assert!(ses.contains("(net \"NET_SIG1\""));
    assert!(ses.contains("(net \"NET_SIG2\""));
}

// ============================================================================
// 9. File-Based High-Level Pipeline Test
// ============================================================================

#[test]
fn test_file_based_route_dsn_pipeline() {
    let tmp_dir = std::env::temp_dir().join("specctra_test_run");
    let _ = fs::create_dir_all(&tmp_dir);

    let dsn_file = tmp_dir.join("input_design.dsn");
    let ses_file = tmp_dir.join("output_session.ses");

    fs::write(&dsn_file, KICAD_DSN_SAMPLE).expect("write temp dsn");

    let report = route_dsn_file_to_ses_file(&dsn_file, &ses_file, None).expect("execute file routing");
    assert_eq!(report.routed_nets, 1);
    assert_eq!(report.unrouted_nets, 0);

    let ses_content = fs::read_to_string(&ses_file).expect("read temp ses");
    assert!(ses_content.contains("(session \"kicad_test_board.dsn\""));
    assert!(ses_content.contains("(routes"));

    // Clean up
    let _ = fs::remove_file(dsn_file);
    let _ = fs::remove_file(ses_file);
    let _ = fs::remove_dir(tmp_dir);
}

// ============================================================================
// 10. Polygon Boundary & Polygon Keepout Ingestion Test
// ============================================================================

#[test]
fn test_polygon_boundary_and_polygon_keepout() {
    let dsn_content = r#"
    (pcb "polygon_board"
      (resolution um 1000)
      (structure
        (layer F.Cu (type signal))
        (layer B.Cu (type signal))
        (boundary (polygon pcb 0 0 40000 0 40000 30000 0 30000))
        (via "Via[0-1]_800:400_um")
        (place_keepout "KO1" (polygon pcb 15000 10000 25000 10000 25000 20000 15000 20000))
        (rule (width 250) (clearance 200))
      )
      (placement
        (component CMP
          (place U1 5000 15000 front 0 (pn "RES_0805"))
          (place U2 35000 15000 front 0 (pn "RES_0805"))
        )
      )
      (library
        (image CMP
          (pin P1 1 0 0)
        )
        (padstack P1
          (shape (rect F.Cu 1000 1000))
        )
      )
      (network
        (net "SIG" (pins U1-1 U2-1))
      )
    )
    "#;

    let dsn = parse_specctra_dsn(dsn_content).expect("parse polygon dsn");
    assert_eq!(dsn.bounds.width(), 40.0);
    assert_eq!(dsn.bounds.height(), 30.0);
    assert_eq!(dsn.keepouts.len(), 1);

    let (mut board, config) = dsn.to_board().expect("ingest polygon board");
    assert_eq!(board.footprints.len(), 3); // U1, U2, and __KEEPOUT_0

    // Component part number verification
    let u1 = board.footprints.iter().find(|f| f.reference == "U1").unwrap();
    assert_eq!(u1.value, "RES_0805");

    // Route around keepout obstacle
    let report = route_board_with_report(&mut board, &config).expect("route around polygon keepout");
    assert_eq!(report.routed_nets, 1);
    assert_eq!(report.unrouted_nets, 0);

    let drc = run_drc(&board, &config);
    assert_eq!(drc.unrouted_nets.len(), 0);
    assert_eq!(drc.clearance_violations.len(), 0);
    assert_eq!(drc.short_circuits.len(), 0);
}

// ============================================================================
// 11. Multi-line Pins & Diverse Pin Format Ingestion Test
// ============================================================================

#[test]
fn test_multiline_pins_and_mixed_syntax() {
    let dsn_content = r#"
    (pcb "multipin_board"
      (resolution um 1000)
      (structure
        (layer F.Cu (type signal))
        (layer B.Cu (type signal))
        (boundary (rect pcb 0 0 50000 50000))
        (via "Via[0-1]_800:400_um")
        (rule (width 250) (clearance 200))
      )
      (placement
        (component IC
          (place U1 10000 10000 front 0)
          (place U2 10000 20000 front 0)
          (place U3 10000 30000 front 0)
          (place U4 10000 40000 front 0)
          (place U5 30000 10000 front 0)
        )
      )
      (library
        (image IC
          (pin P1 1 0 0)
          (pin P1 2 2000 0)
        )
        (padstack P1
          (shape (circle F.Cu 800))
        )
      )
      (network
        (net "BUS"
          (pins "U1"-1 "U2"-2)
          (pins U3-"1" "U4" "-" "2")
          (pins U5 -1)
        )
      )
    )
    "#;

    let dsn = parse_specctra_dsn(dsn_content).expect("parse multipin dsn");
    let net = dsn.nets.iter().find(|n| n.name == "BUS").expect("find BUS net");
    // All 5 pins must be parsed across multiple (pins ...) containers and formats
    assert_eq!(net.pins.len(), 5);
    assert_eq!(net.pins[0], ("U1".to_string(), "1".to_string()));
    assert_eq!(net.pins[1], ("U2".to_string(), "2".to_string()));
    assert_eq!(net.pins[2], ("U3".to_string(), "1".to_string()));
    assert_eq!(net.pins[3], ("U4".to_string(), "2".to_string()));
    assert_eq!(net.pins[4], ("U5".to_string(), "1".to_string()));
}

// ============================================================================
// 12. Net-Class Rule Inheritance Test
// ============================================================================

#[test]
fn test_netclass_rule_inheritance() {
    let dsn_content = r#"
    (pcb "class_test"
      (resolution um 1000)
      (structure
        (layer F.Cu (type signal))
        (layer B.Cu (type signal))
        (boundary (rect pcb 0 0 30000 30000))
        (rule (width 250) (clearance 200))
      )
      (network
        (class POWER VCC GND
          (rule (width 500) (clearance 350))
        )
        (net VCC (pins U1-1 U2-1))
        (net GND (pins U1-2 U2-2))
        (net SIG (pins U1-3 U2-3))
      )
    )
    "#;

    let dsn = parse_specctra_dsn(dsn_content).expect("parse class dsn");
    let vcc = dsn.nets.iter().find(|n| n.name == "VCC").unwrap();
    let gnd = dsn.nets.iter().find(|n| n.name == "GND").unwrap();
    let sig = dsn.nets.iter().find(|n| n.name == "SIG").unwrap();

    // VCC and GND inherit from POWER class
    assert_eq!(vcc.width, Some(0.50));
    assert!((vcc.clearance.unwrap()-0.35).abs()<1e-12);
    assert_eq!(gnd.width, Some(0.50));
    assert!((gnd.clearance.unwrap()-0.35).abs()<1e-12);

    // SIG has default rules
    assert_eq!(sig.width, None);
    assert_eq!(sig.clearance, None);
}

// ============================================================================
// 13. Pad Rotation (Rect / Obround Width & Height Swap) Test
// ============================================================================

#[test]
fn test_pad_rotation_rect_obround() {
    let dsn_content = r#"
    (pcb "rot_test"
      (resolution um 1000)
      (structure
        (layer F.Cu (type signal))
        (boundary (rect pcb 0 0 20000 20000))
      )
      (placement
        (component CMP
          (place U1 10000 10000 front 0)
        )
      )
      (library
        (image CMP
          (pin P_NORM 1 0 0 0)
          (pin P_ROT 2 2000 0 90)
        )
        (padstack P_NORM
          (shape (rect F.Cu 1200 600))
        )
        (padstack P_ROT
          (shape (rect F.Cu 1200 600))
        )
      )
      (network)
    )
    "#;

    let dsn = parse_specctra_dsn(dsn_content).expect("parse rot dsn");
    let (board, _) = dsn.to_board().expect("ingest rot board");
    let u1 = board.footprints.iter().find(|f| f.reference == "U1").unwrap();

    let p_norm = u1.pad("1").unwrap();
    assert_eq!(p_norm.shape, PadShape::Rect { width: 1.2, height: 0.6 });

    let p_rot = u1.pad("2").unwrap();
    // 90-degree rotated pad should have width and height swapped: 0.6 x 1.2
    assert_eq!(p_rot.shape, PadShape::Rect { width: 0.6, height: 1.2 });
}

// ============================================================================
// 14. Keepout Persistence in Bidirectional DSN Export Test
// ============================================================================

#[test]
fn test_keepout_export_roundtrip() {
    let dsn_content = r#"
    (pcb "keepout_rt"
      (resolution um 1000)
      (structure
        (layer F.Cu (type signal))
        (layer B.Cu (type signal))
        (boundary (rect pcb 0 0 50000 50000))
        (keepout "KO_TEST" (rect pcb 20000 20000 30000 30000))
      )
      (placement)
      (library)
      (network)
    )
    "#;

    let dsn = parse_specctra_dsn(dsn_content).expect("parse keepout rt");
    assert_eq!(dsn.keepouts.len(), 1);

    let (board, _) = dsn.to_board().expect("ingest keepout board");
    assert_eq!(board.footprints.len(), 1); // __KEEPOUT_0

    // Export DSN
    let cfg = SpecctraDsnExportConfig {
        pcb_name: "keepout_exported".to_string(),
        unit: SpecctraUnit::Mm,
        resolution_factor: 1000.0,
        top_layer_name: "F.Cu".to_string(),
        bot_layer_name: "B.Cu".to_string(),
    };
    let exported_dsn_text = export_specctra_dsn(&board, &cfg);
    assert!(exported_dsn_text.contains("(keepout \"__KEEPOUT_0\""));
    assert!(exported_dsn_text.contains("(rect pcb 20 20 30 30)"));

    // Parse back
    let re_parsed = parse_specctra_dsn(&exported_dsn_text).expect("re-parse keepout dsn");
    assert_eq!(re_parsed.keepouts.len(), 1);
}

// ============================================================================
// 15. Altium Via Dimension Extraction Accuracy Test
// ============================================================================

#[test]
fn test_altium_via_dimension_extraction() {
    let dsn = parse_specctra_dsn(ALTIUM_DSN_SAMPLE).expect("parse altium dsn");
    let (via_diam, via_drill) = dsn.get_via_dimensions(0.25);

    // Altium sample declares: Via_24_12_mil
    // 24 mil outer diameter = 24 * 0.0254 = 0.6096 mm
    // 12 mil drill = 12 * 0.0254 = 0.3048 mm
    assert!((via_diam - 0.6096).abs() < 1e-4, "Expected ~0.6096mm, got {}", via_diam);
    assert!((via_drill - 0.3048).abs() < 1e-4, "Expected ~0.3048mm, got {}", via_drill);
}

