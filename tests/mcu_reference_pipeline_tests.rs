//! E2E Test Suite: Microcontroller Reference Pipeline & Real-World Scenarios (Tier 4)
//!
//! Verifies:
//! - Scenario 1: Reference MCU Dev Board End-to-End (ATmega328P, USB-C, LDO, Passives, Headers)
//! - Scenario 2: High-Density Microcontroller QFP Breakout & Via Fanout
//! - Scenario 3: Pure Analog / High-Current Power Circuit (Wide traces, High clearance)
//! - Scenario 4: Full CAM Fabrication Package Generation (All 6 Gerber layers + Excellon drill)
//! - Scenario 5: Comprehensive DRC Verification on Complex Reference Workload

use netlist_shatter_viz::pcb::*;
use std::collections::HashMap;

// ============================================================================
// Scenario 1: Reference MCU Dev Board End-to-End Pipeline
// ============================================================================

#[test]
fn test_mcu_reference_board_component_inventory_and_bom() {
    let board = create_reference_board();

    // Verify board boundary outline
    assert!(board.width >= 50.0 && board.width <= 70.0, "Board width must be 50-70mm, got {}", board.width);
    assert!(board.height >= 20.0 && board.height <= 45.0, "Board height must be 20-45mm, got {}", board.height);

    // Verify key integrated circuit components exist in footprint list
    let u1 = board.find_footprint("U1").expect("MCU U1 (ATmega328P) must be placed");
    assert_eq!(u1.pads.len(), 32, "U1 must have 32 pads");

    let u2 = board.find_footprint("U2").expect("LDO U2 (AMS1117) must be placed");
    assert!(u2.pads.len() >= 3, "U2 must have at least 3 pads");

    let j1 = board.find_footprint("J1").expect("USB-C J1 must be placed");
    assert!(j1.pads.len() >= 16, "J1 must have at least 16 pads");

    let y1 = board.find_footprint("Y1").expect("Crystal Y1 must be placed");
    assert!(y1.pads.len() >= 2, "Y1 must have at least 2 pads");

    // Passives
    assert!(board.find_footprint("C1").is_some(), "C1 must be placed");
    assert!(board.find_footprint("C2").is_some(), "C2 must be placed");
    assert!(board.find_footprint("C3").is_some(), "C3 must be placed");
    assert!(board.find_footprint("R1").is_some(), "R1 must be placed");
    assert!(board.find_footprint("SW1").is_some(), "Reset switch SW1 must be placed");

    // Dual 1x15 headers
    let j2 = board.find_footprint("J2").expect("Header J2 must be placed");
    let j3 = board.find_footprint("J3").expect("Header J3 must be placed");
    assert_eq!(j2.pads.len(), 15, "Header J2 must have 15 pins");
    assert_eq!(j3.pads.len(), 15, "Header J3 must have 15 pins");

    // Total pad count across entire reference dev board
    let total_pads: usize = board.footprints.iter().map(|fp| fp.pads.len()).sum();
    assert!(total_pads >= 80, "Reference dev board must contain >= 80 physical pads, found {}", total_pads);
}

#[test]
fn test_mcu_reference_board_netlist_mapping() {
    let board = create_reference_board();

    // Verify critical reference net names exist
    let net_names_set: Vec<&String> = board.net_names.values().collect();
    assert!(net_names_set.iter().any(|&n| n == "GND"), "Netlist must contain 'GND'");
    assert!(net_names_set.iter().any(|&n| n.contains("5V") || n.contains("VBUS")), "Netlist must contain 5V/VBUS net");
    assert!(net_names_set.iter().any(|&n| n.contains("3V3") || n.contains("VCC")), "Netlist must contain 3V3/VCC net");
    assert!(net_names_set.iter().any(|&n| n == "RESET"), "Netlist must contain 'RESET'");
    assert!(net_names_set.iter().any(|&n| n == "XTAL1"), "Netlist must contain 'XTAL1'");
    assert!(net_names_set.iter().any(|&n| n == "XTAL2"), "Netlist must contain 'XTAL2'");

    // Verify GND net is connected to multiple distinct components
    let gnd_id = board.net_names.iter().find(|(_, name)| *name == "GND").map(|(id, _)| *id).expect("GND NetId must exist");

    let mut gnd_connected_components = 0;
    for fp in &board.footprints {
        if fp.pads.iter().any(|pad| pad.net == Some(gnd_id)) {
            gnd_connected_components += 1;
        }
    }
    assert!(
        gnd_connected_components >= 5,
        "GND net must connect to at least 5 distinct components (MCU, USB, LDO, Caps, Headers), found {}",
        gnd_connected_components
    );
}

#[test]
fn test_mcu_reference_board_ratsnest_computation() {
    let board = create_reference_board();
    let ratsnest = compute_ratsnest(&board);

    // Unrouted reference board must have dozens of ratsnest lines
    assert!(
        ratsnest.len() >= 25,
        "Reference MCU dev board must have >= 25 ratsnest lines before routing, found {}",
        ratsnest.len()
    );

    // Invariant: No line connects a pad to itself (zero length)
    for line in &ratsnest {
        let len = line.start.distance_to(line.end);
        assert!(len > 0.001, "Ratsnest must not contain degenerate zero-length loops");
    }

    // Invariant: Every line must have a valid net ID > 0
    for line in &ratsnest {
        assert!(line.net > 0, "Ratsnest line must have a valid non-zero NetId");
    }
}

// ============================================================================
// Scenario 2: High-Density Microcontroller QFP Breakout & Routing
// ============================================================================

#[test]
fn test_qfp_breakout_routing_and_via_fanout() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let u1 = tqfp32("U1", "MCU").at_position(Point::new(25.0, 25.0));
    board.add_footprint(u1);

    // Add 4 destination pads spaced around perimeter
    for i in 1..=4 {
        board.register_net(i, format!("SIG_{}", i));
        // Assign net i to MCU pad i
        if let Some(pad) = board.find_footprint_mut("U1").unwrap().pad_mut(&i.to_string()) {
            pad.net = Some(i);
        }
        let dest_pad = Pad::new_smd("1", "D", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(i);
        let dest_fp = Footprint::new(format!("J{}", i), "HDR", "PKG", Point::new(5.0 * i as f64, 5.0), 0.0, vec![dest_pad], Vec::new());
        board.add_footprint(dest_fp);
    }

    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.20,
        via_diameter: 0.60,
        via_drill: 0.30,
        clearance: 0.15,
    };

    let result = route_board(&mut board, &config);
    assert!(result.is_ok(), "High-density QFP escape routing must succeed");

    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "No short circuits in QFP breakout");
    assert_eq!(drc.clearance_violations.len(), 0, "No clearance violations in QFP breakout");
}

// ============================================================================
// Scenario 3: Pure Analog / High-Current Power Circuit
// ============================================================================

#[test]
fn test_wide_trace_high_clearance_power_routing() {
    let mut board = PcbBoard::new(60.0, 40.0);
    board.register_net(1, "+12V_PWR");
    board.register_net(2, "GND_PWR");

    // High current power pads (size 2.5 x 2.5 mm)
    let p_in_pos = Pad::new_smd("1", "VIN+", Point::new(0.0, 0.0), PadShape::Rect { width: 2.5, height: 2.5 }, Layer::FCu).with_net(1);
    let p_in_neg = Pad::new_smd("1", "VIN-", Point::new(0.0, 0.0), PadShape::Rect { width: 2.5, height: 2.5 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("J_IN+", "PWR", "PWR", Point::new(10.0, 15.0), 0.0, vec![p_in_pos], Vec::new()));
    board.add_footprint(Footprint::new("J_IN-", "PWR", "PWR", Point::new(10.0, 25.0), 0.0, vec![p_in_neg], Vec::new()));

    let p_out_pos = Pad::new_smd("1", "VOUT+", Point::new(0.0, 0.0), PadShape::Rect { width: 2.5, height: 2.5 }, Layer::FCu).with_net(1);
    let p_out_neg = Pad::new_smd("1", "VOUT-", Point::new(0.0, 0.0), PadShape::Rect { width: 2.5, height: 2.5 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("J_OUT+", "PWR", "PWR", Point::new(50.0, 15.0), 0.0, vec![p_out_pos], Vec::new()));
    board.add_footprint(Footprint::new("J_OUT-", "PWR", "PWR", Point::new(50.0, 25.0), 0.0, vec![p_out_neg], Vec::new()));

    // Power router config: wide 1.0mm traces and generous 0.5mm clearance
    let pwr_config = RouterConfig {
        grid_resolution: 0.50,
        default_trace_width: 1.00,
        via_diameter: 1.50,
        via_drill: 0.80,
        clearance: 0.50,
    };

    let result = route_board(&mut board, &pwr_config);
    assert!(result.is_ok(), "Power routing must succeed with wide traces");

    let drc = run_drc(&board, &pwr_config);
    assert_eq!(drc.unrouted_nets.len(), 0, "All power nets must be routed");
    assert_eq!(drc.short_circuits.len(), 0, "No shorts in power routing");
    assert_eq!(drc.clearance_violations.len(), 0, "Wide power clearance must be respected");
}

// ============================================================================
// Scenario 4: Full CAM Fabrication Package Export Validation
// ============================================================================

#[test]
fn test_mcu_reference_board_kicad_export_validity() {
    let board = create_reference_board();
    let kicad_pcb = export_kicad_pcb(&board);

    // Parentheses balance
    let open_count = kicad_pcb.matches('(').count();
    let close_count = kicad_pcb.matches(')').count();
    assert_eq!(open_count, close_count, "KiCad S-expr parentheses must be perfectly balanced");

    // All BOM references must appear in KiCad file
    assert!(kicad_pcb.contains("\"U1\""), "KiCad must include U1");
    assert!(kicad_pcb.contains("\"U2\""), "KiCad must include U2");
    assert!(kicad_pcb.contains("\"J1\""), "KiCad must include J1");
    assert!(kicad_pcb.contains("\"Y1\""), "KiCad must include Y1");
    assert!(kicad_pcb.contains("\"J2\""), "KiCad must include J2");
    assert!(kicad_pcb.contains("\"J3\""), "KiCad must include J3");

    // Edge.Cuts outline
    assert!(kicad_pcb.contains("\"Edge.Cuts\""), "KiCad must include Edge.Cuts");
}

#[test]
fn test_mcu_reference_board_gerber_and_drill_export_package() {
    let board = create_reference_board();

    // Verify all 6 Gerber layers generate non-empty content with valid headers
    let layers = [
        (Layer::FCu, "F_Cu"),
        (Layer::BCu, "B_Cu"),
        (Layer::FSilkscreen, "F_SilkS"),
        (Layer::EdgeCuts, "Edge_Cuts"),
        (Layer::FMask, "F_Mask"),
        (Layer::BMask, "B_Mask"),
    ];

    for (layer, name) in layers {
        let gbr = export_gerber_layer(&board, layer);
        assert!(!gbr.is_empty(), "Gerber layer {} must not be empty", name);
        assert!(gbr.contains("%FSLAX46Y46*%"), "Gerber {} missing %FS% header", name);
        assert!(gbr.contains("%MOMM*%"), "Gerber {} missing %MO% header", name);
        assert!(gbr.contains("%LPD*%"), "Gerber {} missing %LP% header", name);
        assert!(gbr.trim_end().ends_with("M02*"), "Gerber {} must end with M02*", name);
    }

    // Verify Excellon drill export
    let drl = export_excellon_drill(&board);
    assert!(drl.starts_with("M48"), "Drill file must start with M48");
    assert!(drl.contains("METRIC"), "Drill file must specify METRIC");
    assert!(drl.contains("T0"), "Drill file must define tools");
    assert!(drl.trim_end().ends_with("M30"), "Drill file must end with M30");
}

// ============================================================================
// Scenario 5: Full End-to-End Pipeline Execution (Layout -> Route -> DRC -> Export)
// ============================================================================

#[test]
fn test_full_pipeline_layout_to_routing_to_drc_to_export() {
    // 1. Ingest/Create Reference Board
    let mut board = create_reference_board();
    let initial_ratsnest = compute_ratsnest(&board);
    assert!(!initial_ratsnest.is_empty(), "Initial board must have unrouted ratsnest lines");

    // 2. Configure Autorouter
    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.8,
        via_drill: 0.4,
        clearance: 0.2,
    };

    // 3. Execute Autorouting
    let route_result = route_board(&mut board, &config);
    assert!(route_result.is_ok(), "End-to-end autorouting of reference board must succeed");

    // 4. Run Automated DRC
    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "Full pipeline must yield 0 short circuits");
    assert_eq!(drc.clearance_violations.len(), 0, "Full pipeline must yield 0 clearance violations");
    assert_eq!(drc.annular_ring_violations.len(), 0, "Full pipeline must yield 0 annular ring violations");
    assert_eq!(drc.edge_keepout_violations.len(), 0, "Full pipeline must yield 0 edge keepout violations");

    // 5. Generate Fabrication Outputs
    let kicad_out = export_kicad_pcb(&board);
    assert!(kicad_out.contains("(segment"), "Routed board KiCad export must contain track segments");
    assert!(kicad_out.contains("(via"), "Routed board KiCad export must contain vias");

    let fcu_gbr = export_gerber_layer(&board, Layer::FCu);
    assert!(fcu_gbr.contains("D01*"), "Top copper Gerber must contain track draws");

    let drl = export_excellon_drill(&board);
    assert!(drl.contains("X") && drl.contains("Y"), "Drill file must contain hits");
}
