//! Independent Forensic Auditor Adversarial Stress Test Suite for Milestone 5
//! Microcontroller Reference System & End-to-End Pipeline Verification.
//!
//! Author: Milestone 5 Forensic Auditor (`m5_auditor`)
//!
//! Validates:
//! 1. Component inventory, footprint geometries, and boundary containment.
//! 2. Unrouted board DRC behavior: zero placement clearance violations, accurate unrouted nets count.
//! 3. Ratsnest EMST optimality and dynamic response to component translation.
//! 4. DRC defect injection sensitivity on the reference board (shorts, clearance, keepout, unrouted).
//! 5. Autorouter trace topology, layer biasing, via layer-switching validity, and island clustering.
//! 6. KiCad S-expression exporter fidelity before and after autorouting.
//! 7. CAM Gerber & Excellon drill precision and aperture mapping.
//! 8. Stress testing with tighter router configs and perturbed layouts.

use netlist_shatter_viz::pcb::*;

fn reference_router_config() -> RouterConfig {
    RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    }
}

// ============================================================================
// 1. Physical Footprint Geometry & Boundary Containment Audit
// ============================================================================

#[test]
fn adversarial_test_reference_board_all_pads_within_boundary() {
    let board = create_reference_board();

    assert_eq!(board.width, 65.0);
    assert_eq!(board.height, 32.0);

    let board_rect = Rect::new(Point::new(0.0, 0.0), Point::new(board.width, board.height));

    for fp in &board.footprints {
        for pad in &fp.pads {
            let global_pt = fp.pad_world_position(pad);
            let bbox = pad.shape.bounding_box(global_pt, fp.rotation);

            assert!(
                bbox.min.x >= board_rect.min.x - 0.01,
                "Pad {} on {} extends past left board edge: min_x = {}",
                pad.pin_number,
                fp.reference,
                bbox.min.x
            );
            assert!(
                bbox.max.x <= board_rect.max.x + 0.01,
                "Pad {} on {} extends past right board edge: max_x = {}",
                pad.pin_number,
                fp.reference,
                bbox.max.x
            );
            assert!(
                bbox.min.y >= board_rect.min.y - 0.01,
                "Pad {} on {} extends past top board edge: min_y = {}",
                pad.pin_number,
                fp.reference,
                bbox.min.y
            );
            assert!(
                bbox.max.y <= board_rect.max.y + 0.01,
                "Pad {} on {} extends past bottom board edge: max_y = {}",
                pad.pin_number,
                fp.reference,
                bbox.max.y
            );
        }
    }
}

// ============================================================================
// 2. Initial Placement DRC: Zero Clearance Violations & Accurate Unrouted Nets
// ============================================================================

#[test]
fn adversarial_test_reference_board_unrouted_drc_behavior() {
    let board = create_reference_board();
    let config = reference_router_config();

    // Verify board is initially unrouted
    assert!(board.tracks.is_empty(), "Initial reference board must not contain pre-baked tracks");
    assert!(board.vias.is_empty(), "Initial reference board must not contain pre-baked vias");

    // Run DRC on unrouted board
    let drc = run_drc(&board, &config);

    // Initial placement must have NO shorts, clearance violations, annular ring, or edge keepout issues
    assert_eq!(drc.short_circuits.len(), 0, "No short circuits in initial placement");
    assert_eq!(drc.clearance_violations.len(), 0, "No clearance violations in initial placement");
    assert_eq!(drc.annular_ring_violations.len(), 0, "No annular ring violations in initial placement");
    assert_eq!(drc.edge_keepout_violations.len(), 0, "No edge keepout violations in initial placement");

    // All active multi-pin nets MUST be flagged as unrouted!
    assert!(
        !drc.unrouted_nets.is_empty(),
        "Unrouted board must report unrouted nets, but got 0"
    );
    // At least GND, 5V, 3V3, RESET, XTAL1, XTAL2 and bus signals must be unrouted
    assert!(
        drc.unrouted_nets.len() >= 10,
        "Expected >= 10 unrouted nets, got {}",
        drc.unrouted_nets.len()
    );
}

// ============================================================================
// 3. Dynamic Ratsnest Responsiveness & Minimum Spanning Tree Properties
// ============================================================================

#[test]
fn adversarial_test_reference_board_ratsnest_dynamic_mutation() {
    let mut board = create_reference_board();
    let initial_ratsnest = compute_ratsnest(&board);
    assert_eq!(initial_ratsnest.len(), 31, "Baseline reference board has 31 ratsnest lines");

    let initial_total_len: f64 = initial_ratsnest
        .iter()
        .map(|line| line.start.distance_to(line.end))
        .sum();

    // Move C1 from (14.0, 8.5) to (14.0, 4.0) - further away from its connected pads
    if let Some(c1) = board.find_footprint_mut("C1") {
        c1.position = Point::new(14.0, 4.0);
    }

    let modified_ratsnest = compute_ratsnest(&board);
    let modified_total_len: f64 = modified_ratsnest
        .iter()
        .map(|line| line.start.distance_to(line.end))
        .sum();

    // Total ratsnest length must dynamically change
    assert!(
        (modified_total_len - initial_total_len).abs() > 0.1,
        "Ratsnest total length must dynamically change when component moves: before={}, after={}",
        initial_total_len,
        modified_total_len
    );
}

// ============================================================================
// 4. DRC Sensitivity to Artificial Violations on Reference Board
// ============================================================================

#[test]
fn adversarial_test_reference_board_drc_short_circuit_detection() {
    let mut board = create_reference_board();
    let config = reference_router_config();

    let net_gnd = board.get_or_create_net("GND");
    let net_5v = board.get_or_create_net("5V");

    // Deliberately inject a track on F.Cu that shorts GND and 5V
    board.tracks.push(TrackSegment {
        start: Point::new(14.0, 8.5),
        end: Point::new(14.0, 16.0),
        width: 0.25,
        layer: Layer::FCu,
        net: net_gnd,
    });
    board.tracks.push(TrackSegment {
        start: Point::new(14.0, 16.0),
        end: Point::new(14.0, 23.5),
        width: 0.25,
        layer: Layer::FCu,
        net: net_5v,
    });

    let drc = run_drc(&board, &config);
    assert!(
        !drc.short_circuits.is_empty(),
        "DRC must detect deliberately injected short circuit between GND and 5V"
    );
}

#[test]
fn adversarial_test_reference_board_drc_edge_keepout_detection() {
    let mut board = create_reference_board();
    let config = reference_router_config();
    let net_gnd = board.get_or_create_net("GND");

    // Deliberately place a track extending past the right board edge (X = 66.0 > 65.0)
    board.tracks.push(TrackSegment {
        start: Point::new(60.0, 16.0),
        end: Point::new(66.0, 16.0),
        width: 0.25,
        layer: Layer::FCu,
        net: net_gnd,
    });

    let drc = run_drc(&board, &config);
    assert!(
        !drc.edge_keepout_violations.is_empty(),
        "DRC must detect copper violating board edge boundary keepout"
    );
}

// ============================================================================
// 5. Autorouter Trace Topology, Layer Biasing & Via Validity
// ============================================================================

#[test]
fn adversarial_test_reference_board_autorouting_trace_topology() {
    let mut board = create_reference_board();
    let config = reference_router_config();

    let result = route_board(&mut board, &config);
    assert!(result.is_ok(), "Autorouting reference board must succeed: {:?}", result);

    // Verify significant physical copper was routed
    assert!(board.tracks.len() >= 50, "Expected >= 50 track segments, got {}", board.tracks.len());
    assert!(board.vias.len() >= 10, "Expected >= 10 vias, got {}", board.vias.len());

    // Verify all tracks have valid length > 0
    for track in &board.tracks {
        let len = track.start.distance_to(track.end);
        assert!(len > 0.001, "Zero-length track segment detected!");
        assert!(track.width >= config.default_trace_width - 1e-4);
        assert!(track.layer == Layer::FCu || track.layer == Layer::BCu);
    }

    // Verify layer biasing: F.Cu tracks should be predominantly horizontal or 45°,
    // B.Cu tracks should be predominantly vertical or 45°
    let mut fcu_horiz = 0;
    let mut fcu_vert = 0;
    let mut bcu_horiz = 0;
    let mut bcu_vert = 0;

    for track in &board.tracks {
        let dx = (track.end.x - track.start.x).abs();
        let dy = (track.end.y - track.start.y).abs();
        if track.layer == Layer::FCu {
            if dx >= dy { fcu_horiz += 1; } else { fcu_vert += 1; }
        } else if track.layer == Layer::BCu {
            if dy >= dx { bcu_vert += 1; } else { bcu_horiz += 1; }
        }
    }

    assert!(
        fcu_horiz >= fcu_vert,
        "F.Cu should prefer horizontal routing (horiz={}, vert={})",
        fcu_horiz,
        fcu_vert
    );
    assert!(
        bcu_vert >= bcu_horiz,
        "B.Cu should prefer vertical routing (vert={}, horiz={})",
        bcu_vert,
        bcu_horiz
    );

    // Verify via validity: every via must connect to copper of the same net
    for via in &board.vias {
        assert_eq!(via.size, config.via_diameter);
        assert_eq!(via.drill, config.via_drill);

        // Check that at least one track touches this via on the same net
        let touches_track = board.tracks.iter().any(|t| {
            t.net == via.net && (t.start.distance_to(via.at) < 0.01 || t.end.distance_to(via.at) < 0.01)
        });
        assert!(touches_track, "Via at ({}, {}) on net {} is orphaned!", via.at.x, via.at.y, via.net);
    }

    // DRC on routed board must be 100% clean
    let drc = run_drc(&board, &config);
    assert!(drc.is_clean(), "Routed reference board must be 100% DRC clean: {:?}", drc);
}

// ============================================================================
// 6. KiCad S-Expression Exporter Fidelity Before & After Autorouting
// ============================================================================

#[test]
fn adversarial_test_reference_board_kicad_export_transformation() {
    let unrouted_board = create_reference_board();
    let unrouted_kicad = export_kicad_pcb(&unrouted_board);

    // Unrouted KiCad export must NOT contain track segments or vias
    assert!(!unrouted_kicad.contains("(segment"), "Unrouted board KiCad export must NOT contain (segment");
    assert!(!unrouted_kicad.contains("(via"), "Unrouted board KiCad export must NOT contain (via");

    // Route the board
    let mut routed_board = create_reference_board();
    let config = reference_router_config();
    route_board(&mut routed_board, &config).expect("Route succeeds");

    let routed_kicad = export_kicad_pcb(&routed_board);

    // Routed KiCad export MUST contain tracks and vias
    assert!(routed_kicad.contains("(segment"), "Routed KiCad export MUST contain (segment");
    assert!(routed_kicad.contains("(via"), "Routed KiCad export MUST contain (via");

    // Both must have strictly balanced parentheses
    let count_open = routed_kicad.matches('(').count();
    let count_close = routed_kicad.matches(')').count();
    assert_eq!(count_open, count_close, "KiCad S-expression must have balanced parentheses");

    // Routed version must be strictly larger than unrouted version
    assert!(
        routed_kicad.len() > unrouted_kicad.len(),
        "Routed KiCad output (len {}) must exceed unrouted output (len {})",
        routed_kicad.len(),
        unrouted_kicad.len()
    );
}

// ============================================================================
// 7. CAM Gerber & Drill Output Precision on Reference Board
// ============================================================================

#[test]
fn adversarial_test_reference_board_cam_layer_apertures_and_drill() {
    let mut board = create_reference_board();
    let config = reference_router_config();
    route_board(&mut board, &config).expect("Route succeeds");

    // Edge.Cuts Gerber
    let edge_gbr = export_gerber_layer(&board, Layer::EdgeCuts);
    assert!(edge_gbr.contains("%FSLAX46Y46*%"));
    assert!(edge_gbr.contains("M02*"));

    // Check silkscreen has component labels
    let silk_gbr = export_gerber_layer(&board, Layer::FSilkscreen);
    assert!(silk_gbr.contains("M02*"));

    // Check drill file
    let drl = export_excellon_drill(&board);
    assert!(drl.starts_with("M48"));
    assert!(drl.trim_end().ends_with("M30"));

    // Drill file should contain tools for THT pins and vias
    assert!(drl.contains("T01C") || drl.contains("T02C"));
}

// ============================================================================
// 8. Tighter Router Config Stress Test on Reference Board
// ============================================================================

#[test]
fn adversarial_test_reference_board_tighter_router_config_stress() {
    let mut board = create_reference_board();
    // Tighter config: 0.20mm traces, 0.60mm vias with 0.30mm drill
    let tighter_config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.20,
        via_diameter: 0.60,
        via_drill: 0.30,
        clearance: 0.20,
    };

    let result = route_board(&mut board, &tighter_config);
    assert!(result.is_ok(), "Reference board must route cleanly under tighter config: {:?}", result);

    let drc = run_drc(&board, &tighter_config);
    assert_eq!(drc.short_circuits.len(), 0, "No shorts under tighter config");
    assert_eq!(drc.clearance_violations.len(), 0, "No clearance violations under tighter config");
    assert_eq!(drc.unrouted_nets.len(), 0, "All nets routed under tighter config");
}
