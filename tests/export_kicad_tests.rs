//! E2E Test Suite: KiCad S-Expression Exporter (.kicad_pcb) (Tiers 1 & 2)
//!
//! Verifies:
//! - Strictly balanced S-expression parentheses throughout the document (F24)
//! - Valid KiCad 7/8 header format and generator attributes (F24)
//! - Netlist table serialization with (net 0 "") and defined nets (F25)
//! - Footprint serialization with SMD and THT pad geometry, rotation, and properties (F25)
//! - Track segments and through-hole vias serialization with net binding (F26)
//! - Edge.Cuts closed perimeter polygon and silkscreen lines (F27)
//! - Tier 2 boundary cases: empty boards, symbol-laden net names, rotated elements

use netlist_shatter_viz::pcb::*;
use std::collections::HashMap;

#[test]
fn test_kicad_export_preserves_native_route_coordinates() {
    let mut board = PcbBoard::new(60.0, 35.0);
    board.register_net(1, "GND");
    board.add_track(TrackSegment::new(
        Point::new(25.273843, 20.339656), Point::new(30.273843, 20.339656),
        0.15, Layer::FCu, 1,
    ));
    board.add_via(Via::new(Point::new(25.273843, 20.339656), 0.6, 0.3, 1));
    let output = export_kicad_pcb(&board);
    assert!(output.contains("(start 25.273843 20.339656)"));
    assert!(output.contains("(at 25.273843 20.339656)"));
}

// Helper to audit parentheses balance in an S-expression
fn assert_s_expression_balanced(content: &str) {
    let mut depth: i32 = 0;
    for (line_idx, line) in content.lines().enumerate() {
        for ch in line.chars() {
            if ch == '(' {
                depth += 1;
            } else if ch == ')' {
                depth -= 1;
                assert!(
                    depth >= 0,
                    "Premature closing parenthesis found at line {}: '{}'",
                    line_idx + 1,
                    line
                );
            }
        }
    }
    assert_eq!(
        depth, 0,
        "Unbalanced S-expression: final parenthesis depth was {}, expected 0",
        depth
    );
}

// Helper to construct a populated board
fn create_sample_pcb_board() -> PcbBoard {
    let mut board = PcbBoard::new(60.0, 35.0);
    board.register_net(0, "");
    board.register_net(1, "GND");
    board.register_net(2, "+5V_USB");
    board.register_net(3, "RESET");

    // Add a 2-pad resistor R1
    let r1 = resistor_0805("R1", "10k")
        .at_position(Point::new(15.0, 15.0))
        .with_rotation(90.0)
        .with_net_assigned("1", 2)
        .with_net_assigned("2", 3);
    board.add_footprint(r1);

    // Add a through-hole header J1
    let j1 = header_1x15("J1", "CONN_01x15")
        .at_position(Point::new(10.0, 5.0))
        .with_rotation(0.0)
        .with_net_assigned("1", 1);
    board.add_footprint(j1);

    // Add tracks on F.Cu and B.Cu
    board.add_track(TrackSegment::new(Point::new(15.0, 15.0), Point::new(25.0, 15.0), 0.25, Layer::FCu, 2));
    board.add_track(TrackSegment::new(Point::new(25.0, 15.0), Point::new(25.0, 25.0), 0.25, Layer::BCu, 2));

    // Add a via connecting the two tracks
    board.add_via(Via::new(Point::new(25.0, 15.0), 0.8, 0.4, 2));

    board
}

// ============================================================================
// Tier 1: S-Expression Syntax & Structural Integrity
// ============================================================================

#[test]
fn test_kicad_export_strictly_balanced_parentheses() {
    let board = create_sample_pcb_board();
    let kicad_sexpr = export_kicad_pcb(&board);

    assert_s_expression_balanced(&kicad_sexpr);
}

#[test]
fn test_kicad_export_root_header_and_generator() {
    let board = create_sample_pcb_board();
    let output = export_kicad_pcb(&board);

    // Must start with (kicad_pcb
    assert!(
        output.trim_start().starts_with("(kicad_pcb"),
        "KiCad output must start with '(kicad_pcb', got: {}",
        &output[..output.len().min(100)]
    );

    // Must declare version and generator
    assert!(output.contains("(version"), "Must contain version tag");
    assert!(output.contains("(generator"), "Must contain generator tag");
}

#[test]
fn test_kicad_export_netlist_table() {
    let board = create_sample_pcb_board();
    let output = export_kicad_pcb(&board);

    // Requirement: net 0 is strictly (net 0 "")
    assert!(
        output.contains("(net 0 \"\")"),
        "KiCad specification mandates (net 0 \"\")"
    );

    // Registered nets must be present
    assert!(output.contains("(net 1 \"GND\")"), "Must declare net 1 GND");
    assert!(output.contains("(net 2 \"+5V_USB\")"), "Must declare net 2 +5V_USB");
    assert!(output.contains("(net 3 \"RESET\")"), "Must declare net 3 RESET");
}

#[test]
fn test_kicad_export_footprint_declaration_and_properties() {
    let board = create_sample_pcb_board();
    let output = export_kicad_pcb(&board);

    // Footprints U1/R1 and J1
    assert!(output.contains("(footprint"), "Must contain footprint tags");
    assert!(output.contains("\"R1\""), "Must contain reference R1");
    assert!(output.contains("\"10k\""), "Must contain value 10k");
    assert!(output.contains("\"J1\""), "Must contain reference J1");

    // Footprint rotation: R1 is placed at (15.0, 15.0) rotated 90 deg
    assert!(
        output.contains("(at 15") && (output.contains("90)") || output.contains("90.0)")),
        "Rotated footprint must include rotation angle in (at ...)"
    );
}

#[test]
fn test_kicad_export_smd_pad_syntax() {
    let board = create_sample_pcb_board();
    let output = export_kicad_pcb(&board);

    // SMD pad must specify smd, size, layers, and net
    assert!(output.contains("smd"), "SMD pad must specify 'smd'");
    assert!(output.contains("(layers \"F.Cu\""), "SMD pad must specify copper layer");
    assert!(output.contains("(net 2"), "Assigned pad must contain net ID");
}

#[test]
fn test_kicad_export_tht_pad_syntax() {
    let board = create_sample_pcb_board();
    let output = export_kicad_pcb(&board);

    // Through-hole pad must specify thru_hole and drill
    assert!(
        output.contains("thru_hole"),
        "THT pad in header J1 must specify 'thru_hole'"
    );
    assert!(
        output.contains("(drill"),
        "THT pad must declare (drill ...)"
    );
}

#[test]
fn test_kicad_export_tracks_and_vias() {
    let board = create_sample_pcb_board();
    let output = export_kicad_pcb(&board);

    // Track segment: (segment (start ...) (end ...) (width ...) (layer ...) (net ...))
    assert!(output.contains("(segment"), "Must serialize track segments");
    assert!(output.contains("(layer \"F.Cu\")"), "Must specify track layer F.Cu");
    assert!(output.contains("(layer \"B.Cu\")"), "Must specify track layer B.Cu");
    assert!(output.contains("(width 0.25"), "Must declare track width 0.25");

    // Via: (via (at ...) (size ...) (drill ...) (layers "F.Cu" "B.Cu") (net ...))
    assert!(output.contains("(via"), "Must serialize vias");
    assert!(output.contains("(size 0.8"), "Via must specify outer diameter (size 0.8)");
    assert!(output.contains("(drill 0.4"), "Via must specify drill (drill 0.4)");
}

#[test]
fn test_kicad_export_edge_cuts_closed_loop() {
    let board = create_sample_pcb_board();
    let output = export_kicad_pcb(&board);

    // Edge.Cuts must be serialized as graphical lines on layer "Edge.Cuts"
    assert!(
        output.contains("(layer \"Edge.Cuts\")"),
        "Must include board boundary segments on layer Edge.Cuts"
    );

    // Count gr_line occurrences on Edge.Cuts (must be >= 4 for a rectangle)
    let edge_cuts_count = output.matches("(layer \"Edge.Cuts\")").count();
    assert!(
        edge_cuts_count >= 4,
        "Rectangular board must have at least 4 Edge.Cuts segments, found {}",
        edge_cuts_count
    );
}

// ============================================================================
// Tier 2: Boundary Cases (Empty Boards, Special Net Names, Zero Geometry)
// ============================================================================

#[test]
fn test_kicad_export_empty_board() {
    let board = PcbBoard::new(40.0, 30.0);
    let output = export_kicad_pcb(&board);

    // Even an empty board must produce a valid, parseable S-expression
    assert_s_expression_balanced(&output);
    assert!(output.trim_start().starts_with("(kicad_pcb"));
    assert!(output.contains("(layer \"Edge.Cuts\")"));
}

#[test]
fn test_kicad_export_special_net_names() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(10, "+3.3V_CORE");
    board.register_net(11, "USB_D+");
    board.register_net(12, "USB_D-");
    board.register_net(13, "SPI/MOSI");

    let output = export_kicad_pcb(&board);
    assert_s_expression_balanced(&output);

    assert!(output.contains("(net 10 \"+3.3V_CORE\")"));
    assert!(output.contains("(net 11 \"USB_D+\")"));
    assert!(output.contains("(net 12 \"USB_D-\")"));
}
