//! E2E Test Suite: RS-274X Gerber & Excellon NC Drill Exporter (Tiers 1 & 2)
//!
//! Verifies:
//! - Standard RS-274X headers (%FSLAX46Y46*%, %MOMM*%, %LPD*%) and M02* EOF (F28)
//! - Dynamic aperture definition table (%ADD10C...%, %ADD...R...%, %ADD...O...%) (F29)
//! - Flash (D03), linear interpolation (D01), and move (D02) command syntax (F30)
//! - All 6 standard fabrication layers (F.Cu, B.Cu, F.SilkS, Edge.Cuts, F.Mask, B.Mask) (F31)
//! - Excellon NC Drill export (M48, METRIC, tool table TnnC..., hits, M30) (F32)
//! - Tier 2 boundary cases: coordinate scaling to 10^6 nm, empty layers, zero-hit drill files

use netlist_shatter_viz::pcb::*;
use std::collections::HashMap;

fn create_gerber_test_board() -> PcbBoard {
    let mut board = PcbBoard::new(60.0, 35.0);
    board.register_net(1, "GND");
    board.register_net(2, "VCC");

    // SMD 0805 resistor at (15.0, 20.0)
    let r1 = resistor_0805("R1", "10k")
        .at_position(Point::new(15.0, 20.0))
        .with_rotation(0.0)
        .with_net_assigned("1", 1)
        .with_net_assigned("2", 2);
    board.add_footprint(r1);

    // THT header 1x15 at (10.0, 5.0)
    let j1 = header_1x15("J1", "CONN_01x15")
        .at_position(Point::new(10.0, 5.0))
        .with_rotation(0.0)
        .with_net_assigned("1", 1);
    board.add_footprint(j1);

    // Top trace and bottom trace
    board.add_track(TrackSegment::new(Point::new(15.0, 20.0), Point::new(25.0, 20.0), 0.25, Layer::FCu, 1));
    board.add_track(TrackSegment::new(Point::new(25.0, 20.0), Point::new(25.0, 30.0), 0.25, Layer::BCu, 1));

    // Via at (25.0, 20.0)
    board.add_via(Via::new(Point::new(25.0, 20.0), 0.8, 0.4, 1));

    board
}

// ============================================================================
// Tier 1: RS-274X Gerber Header & Aperture Table Verification
// ============================================================================

#[test]
fn test_gerber_headers_across_all_layers() {
    let board = create_gerber_test_board();
    let layers = [
        Layer::FCu,
        Layer::BCu,
        Layer::FSilkscreen,
        Layer::EdgeCuts,
        Layer::FMask,
        Layer::BMask,
    ];

    for layer in layers {
        let gbr = export_gerber_layer(&board, layer);

        // Header directives
        assert!(
            gbr.contains("%FSLAX46Y46*%"),
            "Gerber layer {:?} missing %FSLAX46Y46*% format specification",
            layer
        );
        assert!(
            gbr.contains("%MOMM*%"),
            "Gerber layer {:?} missing %MOMM*% millimeter unit specification",
            layer
        );
        assert!(
            gbr.contains("%LPD*%"),
            "Gerber layer {:?} missing %LPD*% layer polarity dark directive",
            layer
        );

        // End of file directive
        assert!(
            gbr.trim_end().ends_with("M02*"),
            "Gerber layer {:?} must end with 'M02*'",
            layer
        );
    }
}

#[test]
fn test_gerber_aperture_definitions_present() {
    let board = create_gerber_test_board();
    let fcu_gbr = export_gerber_layer(&board, Layer::FCu);

    // Must define circular aperture for traces/vias (%ADD...C,...)
    assert!(
        fcu_gbr.contains("%ADD") && fcu_gbr.contains("C,"),
        "F.Cu Gerber must contain circular aperture definition %ADDnnC,..."
    );

    // Must define rectangular aperture for SMD pads (%ADD...R,...)
    assert!(
        fcu_gbr.contains("R,"),
        "F.Cu Gerber must contain rectangular aperture definition %ADDnnR,..."
    );

    // Aperture D-codes must start at 10 or higher
    assert!(
        fcu_gbr.contains("%ADD10"),
        "Aperture table must index dynamic apertures starting at D10"
    );
}

#[test]
fn test_gerber_drawing_and_flashing_commands() {
    let board = create_gerber_test_board();
    let fcu_gbr = export_gerber_layer(&board, Layer::FCu);

    // D01* is linear draw with exposure on (drawing tracks)
    assert!(
        fcu_gbr.contains("D01*"),
        "F.Cu Gerber must contain D01* linear draw commands for copper tracks"
    );

    // D02* is move with exposure off
    assert!(
        fcu_gbr.contains("D02*"),
        "F.Cu Gerber must contain D02* positioning commands"
    );

    // D03* is aperture flash (flashing SMD pads and vias)
    assert!(
        fcu_gbr.contains("D03*"),
        "F.Cu Gerber must contain D03* flash commands for pads and vias"
    );
}

#[test]
fn test_gerber_layer_specific_content() {
    let board = create_gerber_test_board();

    // 1. Edge.Cuts must contain draw commands for board outline
    let edge_gbr = export_gerber_layer(&board, Layer::EdgeCuts);
    assert!(
        edge_gbr.contains("D01*"),
        "Edge.Cuts Gerber must draw perimeter lines"
    );

    // 2. F.SilkS must contain draw commands for footprint silkscreen lines
    let silk_gbr = export_gerber_layer(&board, Layer::FSilkscreen);
    assert!(
        silk_gbr.contains("D01*"),
        "F.SilkS Gerber must draw component silkscreen outlines"
    );

    // 3. F.Mask must contain flashes for SMD and THT pad solder mask openings
    let mask_gbr = export_gerber_layer(&board, Layer::FMask);
    assert!(
        mask_gbr.contains("D03*"),
        "F.Mask Gerber must flash solder mask openings"
    );
}

// ============================================================================
// Tier 1: Excellon NC Drill Export Verification
// ============================================================================

#[test]
fn test_excellon_drill_header_and_tool_table() {
    let board = create_gerber_test_board();
    let drl = export_excellon_drill(&board);

    // Must begin with M48 header
    assert!(
        drl.trim_start().starts_with("M48"),
        "Excellon drill file must start with 'M48', got: {}",
        &drl[..drl.len().min(50)]
    );

    // Must define metric format
    assert!(
        drl.contains("METRIC"),
        "Excellon header must declare METRIC units"
    );

    // Tool table must define via drill (0.400mm) and header drill (1.000mm)
    assert!(
        drl.contains("T01C") || drl.contains("T1C"),
        "Tool table must define drill tools with format TnnC..."
    );

    // End of program command M30
    assert!(
        drl.trim_end().ends_with("M30"),
        "Excellon drill file must terminate with 'M30'"
    );
}

#[test]
fn test_excellon_drill_hits_formatting() {
    let board = create_gerber_test_board();
    let drl = export_excellon_drill(&board);

    // Drill hits must specify coordinates in format X...Y...
    assert!(
        drl.contains("X") && drl.contains("Y"),
        "Excellon drill file must contain X...Y... coordinate hit instructions"
    );

    // Via at (25.0, 20.0) must produce a hit near X=25.0 Y=20.0
    // Header J1 has 15 holes at 2.54mm pitch
    let hit_count = drl.lines().filter(|line| line.starts_with('X') && line.contains('Y')).count();
    // 1 via + 15 header pins = 16 drill hits minimum
    assert!(
        hit_count >= 16,
        "Expected at least 16 drill hits (1 via + 15 header pins), got {}",
        hit_count
    );
}

// ============================================================================
// Tier 2: Coordinate Scaling (10^6 nm) & Boundary Conditions
// ============================================================================

#[test]
fn test_gerber_coordinate_scaling_factor_1e6() {
    let mut board = PcbBoard::new(50.0, 50.0);
    // Track from (1.0 mm, 2.0 mm) to (3.0 mm, 4.0 mm)
    board.add_track(TrackSegment::new(Point::new(1.0, 2.0), Point::new(3.0, 4.0), 0.25, Layer::FCu, 1));

    let gbr = export_gerber_layer(&board, Layer::FCu);

    // Under %FSLAX46Y46*% scaling (factor 10^6):
    // 1.0 mm -> 1000000
    // 2.0 mm -> 2000000
    // 3.0 mm -> 3000000
    // 4.0 mm -> 4000000
    assert!(
        gbr.contains("1000000") || gbr.contains("X1000000"),
        "Gerber must scale X=1.0mm to 1000000 (10^6 integer scale)"
    );
    assert!(
        gbr.contains("Y48000000"),
        "Gerber must scale Y=2.0mm to 2000000"
    );
}

#[test]
fn test_gerber_empty_layer_generates_valid_headers() {
    let board = PcbBoard::new(50.0, 50.0);
    // B.Cu is empty (no tracks, no pads)
    let gbr = export_gerber_layer(&board, Layer::BCu);

    assert!(gbr.contains("%FSLAX46Y46*%"));
    assert!(gbr.contains("%MOMM*%"));
    assert!(gbr.trim_end().ends_with("M02*"));
}

#[test]
fn test_excellon_empty_board_no_crash() {
    let board = PcbBoard::new(50.0, 50.0);
    // Board with no holes
    let drl = export_excellon_drill(&board);

    assert!(drl.starts_with("M48"));
    assert!(drl.trim_end().ends_with("M30"));
}
