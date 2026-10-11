//! Milestone 3 Empirical Challenger Test Suite
//!
//! Stress-tests:
//! 1. Multi-net crossing meshes (4x4 = 16 crossing points) requiring vias & 2-layer switches.
//! 2. Diagonal crossing nets (45° intersection) with automated via placement.
//! 3. Dense parallel buses (8-bit bus with 1.0mm pitch).
//! 4. 45° octilinear jogging bus with parallel clearance maintenance.
//! 5. Obstacle detour & rip-up reroute resolution.
//! 6. Unsolvable/blocked pin termination: verify bounded loop & proper error return.
//! 7. Intertwined multi-pin nets (3-pin and 4-pin) with DRC Union-Find validation.
//! 8. Mixed layer routing (B.Cu-B.Cu, B.Cu-F.Cu, THT-B.Cu, THT-F.Cu).
//! 9. Corner pinching avoidance around diagonal obstacle pads.
//! 10. Performance scaling on multi-net congested boards.

use netlist_shatter_viz::pcb::*;
use std::time::Instant;

fn test_config() -> RouterConfig {
    RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    }
}

// ============================================================================
// Challenge 1: 4x4 Congested Mesh (16 Crossing Points)
// ============================================================================

#[test]
fn test_challenge_dense_mesh_sixteen_intersections() {
    // 4 horizontal nets and 4 vertical nets crossing each other in a 4x4 matrix
    // Horizontal nets: y = 15, 20, 25, 30 across x in [10, 40]
    // Vertical nets:   x = 15, 20, 25, 30 across y in [10, 40]
    // There are 16 intersection points! On 2 layers, this creates heavy routing congestion.
    let mut board = PcbBoard::new(50.0, 50.0);
    let config = test_config();

    // Horizontal nets 1..=4
    for i in 0..4 {
        let net_id = (i + 1) as NetId;
        board.register_net(net_id, format!("H_NET_{}", net_id));
        let y = 15.0 + (i as f64) * 5.0;
        let p_start = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(net_id);
        let p_end = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(net_id);
        board.add_footprint(Footprint::new(format!("UH_S{}", net_id), "P", "P", Point::new(10.0, y), 0.0, vec![p_start], Vec::new()));
        board.add_footprint(Footprint::new(format!("UH_E{}", net_id), "P", "P", Point::new(40.0, y), 0.0, vec![p_end], Vec::new()));
    }

    // Vertical nets 5..=8
    for i in 0..4 {
        let net_id = (i + 5) as NetId;
        board.register_net(net_id, format!("V_NET_{}", net_id));
        let x = 15.0 + (i as f64) * 5.0;
        let p_start = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(net_id);
        let p_end = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(net_id);
        board.add_footprint(Footprint::new(format!("UV_S{}", net_id), "P", "P", Point::new(x, 10.0), 0.0, vec![p_start], Vec::new()));
        board.add_footprint(Footprint::new(format!("UV_E{}", net_id), "P", "P", Point::new(x, 40.0), 0.0, vec![p_end], Vec::new()));
    }

    let report = route_board_with_report(&mut board, &config)
        .expect("Router must successfully route 4x4 crossing mesh");

    assert_eq!(report.routed_nets, 8, "All 8 nets must be routed");
    assert_eq!(report.unrouted_nets, 0);
    assert!(report.total_vias > 0, "Crossing mesh must insert vias for layer transitions");

    // Octilinear invariant check
    for seg in &board.tracks {
        let dx = (seg.end.x - seg.start.x).abs();
        let dy = (seg.end.y - seg.start.y).abs();
        assert!(
            dy < 1e-4 || dx < 1e-4 || (dx - dy).abs() < 1e-4,
            "Non-octilinear track in mesh: ({},{}) to ({},{})",
            seg.start.x, seg.start.y, seg.end.x, seg.end.y
        );
    }

    // DRC MUST BE 100% CLEAN
    let drc = run_drc(&board, &config);
    assert!(drc.is_clean(), "DRC must be clean on 4x4 mesh. Violations: shorts={}, clearance={}, unrouted={}",
        drc.short_circuits.len(), drc.clearance_violations.len(), drc.unrouted_nets.len());
}

// ============================================================================
// Challenge 2: Diagonal Crossing Nets (45° Intersection)
// ============================================================================

#[test]
fn test_challenge_diagonal_crossing_nets() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let config = test_config();

    board.register_net(1, "DIAG_1");
    board.register_net(2, "DIAG_2");

    // Net 1: (10, 10) to (40, 40)
    let p1_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let p1_b = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "P", "P", Point::new(10.0, 10.0), 0.0, vec![p1_a], Vec::new()));
    board.add_footprint(Footprint::new("U2", "P", "P", Point::new(40.0, 40.0), 0.0, vec![p1_b], Vec::new()));

    // Net 2: (10, 40) to (40, 10)
    let p2_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    let p2_b = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U3", "P", "P", Point::new(10.0, 40.0), 0.0, vec![p2_a], Vec::new()));
    board.add_footprint(Footprint::new("U4", "P", "P", Point::new(40.0, 10.0), 0.0, vec![p2_b], Vec::new()));

    route_board(&mut board, &config).expect("Diagonal crossing nets must route");

    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "Diagonal nets must not short");
    assert_eq!(drc.clearance_violations.len(), 0, "Diagonal nets must maintain clearance");
    assert_eq!(drc.unrouted_nets.len(), 0, "Both diagonal nets must be routed");
}

// ============================================================================
// Challenge 3: Dense 8-Bit Parallel Bus (1.0mm Pitch)
// ============================================================================

#[test]
fn test_challenge_dense_eight_bit_parallel_bus() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let config = test_config();

    // 8 parallel lines from x=10 to x=40, spaced by 1.0mm in y: y in [15.0..22.0]
    // Using an 8-pin connector footprint for start and end (reflecting standard 1mm pitch connector)
    let mut start_pads = Vec::new();
    let mut end_pads = Vec::new();

    for i in 0..8 {
        let net_id = (i + 1) as NetId;
        board.register_net(net_id, format!("BUS_{}", net_id));
        let y_rel = (i as f64) * 1.0;
        let p_start = Pad::new_smd(format!("{}", i + 1), "A", Point::new(0.0, y_rel), PadShape::Rect { width: 0.6, height: 0.6 }, Layer::FCu).with_net(net_id);
        let p_end = Pad::new_smd(format!("{}", i + 1), "B", Point::new(0.0, y_rel), PadShape::Rect { width: 0.6, height: 0.6 }, Layer::FCu).with_net(net_id);
        start_pads.push(p_start);
        end_pads.push(p_end);
    }

    board.add_footprint(Footprint::new("J_START", "CONN8", "CONN_1X8", Point::new(10.0, 15.0), 0.0, start_pads, Vec::new()));
    board.add_footprint(Footprint::new("J_END", "CONN8", "CONN_1X8", Point::new(40.0, 15.0), 0.0, end_pads, Vec::new()));

    let report = route_board_with_report(&mut board, &config).expect("8-bit parallel bus must route");
    assert_eq!(report.routed_nets, 8);
    assert_eq!(report.unrouted_nets, 0);

    let drc = run_drc(&board, &config);
    if !drc.is_clean() {
        eprintln!("DRC Violations in 8-bit bus:");
        for v in &drc.clearance_violations {
            eprintln!("  Clearance: {:?} at {:?} msg: {}", v.kind, v.location, v.message);
        }
        for s in &drc.short_circuits {
            eprintln!("  Short: {:?} at {:?} msg: {}", s.kind, s.location, s.message);
        }
        for u in &drc.unrouted_nets {
            eprintln!("  Unrouted net: {}", u);
        }
    }
    assert!(drc.is_clean(), "8-bit parallel bus must have zero DRC violations");
}

// ============================================================================
// Challenge 4: Parallel Bus with 45° Bends
// ============================================================================

#[test]
fn test_challenge_bus_with_45_degree_jog() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let config = test_config();

    // 4 parallel traces: start at (10.0, y), end at (40.0, y + 6.0)
    // Forced 45-degree transition in the middle
    for i in 0..4 {
        let net_id = (i + 1) as NetId;
        board.register_net(net_id, format!("JOG_{}", net_id));
        let y_start = 12.0 + (i as f64) * 1.5;
        let y_end = y_start + 6.0;
        let p_start = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(net_id);
        let p_end = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(net_id);
        board.add_footprint(Footprint::new(format!("UJ_S{}", net_id), "P", "P", Point::new(10.0, y_start), 0.0, vec![p_start], Vec::new()));
        board.add_footprint(Footprint::new(format!("UJ_E{}", net_id), "P", "P", Point::new(40.0, y_end), 0.0, vec![p_end], Vec::new()));
    }

    route_board(&mut board, &config).expect("Jogging bus must route");

    let drc = run_drc(&board, &config);
    assert!(drc.is_clean(), "Jogging bus must pass DRC with 0 errors");
}

// ============================================================================
// Challenge 5: Obstacle Detour & Rip-Up / Reroute
// ============================================================================

#[test]
fn test_challenge_detour_around_foreign_pad_array() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let config = test_config();

    // Net 1 is a passive obstacle net: a wall of 3 pads in the center at x=25, y=18, 20, 22
    board.register_net(1, "WALL");
    let p_w1 = Pad::new_smd("1", "W1", Point::new(0.0, 0.0), PadShape::Rect { width: 1.5, height: 1.5 }, Layer::FCu).with_net(1);
    let p_w2 = Pad::new_smd("2", "W2", Point::new(0.0, 0.0), PadShape::Rect { width: 1.5, height: 1.5 }, Layer::FCu).with_net(1);
    let p_w3 = Pad::new_smd("3", "W3", Point::new(0.0, 0.0), PadShape::Rect { width: 1.5, height: 1.5 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("W1", "P", "P", Point::new(25.0, 18.0), 0.0, vec![p_w1], Vec::new()));
    board.add_footprint(Footprint::new("W2", "P", "P", Point::new(25.0, 20.0), 0.0, vec![p_w2], Vec::new()));
    board.add_footprint(Footprint::new("W3", "P", "P", Point::new(25.0, 22.0), 0.0, vec![p_w3], Vec::new()));

    // Net 2 wants to go from (10.0, 20.0) to (40.0, 20.0) right through the wall
    board.register_net(2, "SIGNAL");
    let p_sig_a = Pad::new_smd("1", "S1", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    let p_sig_b = Pad::new_smd("2", "S2", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("SIG1", "P", "P", Point::new(10.0, 20.0), 0.0, vec![p_sig_a], Vec::new()));
    board.add_footprint(Footprint::new("SIG2", "P", "P", Point::new(40.0, 20.0), 0.0, vec![p_sig_b], Vec::new()));

    route_board(&mut board, &config).expect("Signal net must route around or jump over obstacle wall");

    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "No short circuit with obstacle wall");
    assert_eq!(drc.clearance_violations.len(), 0, "No clearance violation with obstacle wall");
    assert_eq!(drc.unrouted_nets.len(), 0, "All nets routed");
}

// ============================================================================
// Challenge 6: Unsolvable Problem Bounded Termination & Error Handling
// ============================================================================

#[test]
fn test_challenge_unsolvable_fully_boxed_pad_terminates_gracefully() {
    let mut board = PcbBoard::new(40.0, 40.0);
    let config = test_config();

    // Net 1 is our trapped net: pin at (20.0, 20.0)
    board.register_net(1, "TRAPPED");
    let p1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let p2 = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U_TRAP", "P", "P", Point::new(20.0, 20.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U_OUT", "P", "P", Point::new(35.0, 35.0), 0.0, vec![p2], Vec::new()));

    // Surround pin 1 with a tight fortress of 8 foreign THT pads (net 2)
    // THT pads block BOTH F.Cu and B.Cu and prevent via escape!
    board.register_net(2, "FORTRESS");
    let deltas = [
        (-2.0, -2.0), (0.0, -2.0), (2.0, -2.0),
        (-2.0,  0.0),              (2.0,  0.0),
        (-2.0,  2.0), (0.0,  2.0), (2.0,  2.0),
    ];
    for (idx, (dx, dy)) in deltas.iter().enumerate() {
        let p_wall = Pad::new_tht(format!("W{}", idx), "W", Point::new(0.0, 0.0), PadShape::Circle { diameter: 1.8 }, 0.6, 1.8).with_net(2);
        board.add_footprint(Footprint::new(
            format!("WALL_{}", idx), "P", "P",
            Point::new(20.0 + dx, 20.0 + dy), 0.0,
            vec![p_wall], Vec::new(),
        ));
    }

    let start_time = Instant::now();
    let result = route_board(&mut board, &config);
    let elapsed = start_time.elapsed();

    // MUST return Err (UnroutableNet, NoPathFound, or BoardTooDense)
    assert!(result.is_err(), "Trapped net must return an error");
    // MUST terminate quickly without looping indefinitely (budget < 1.0s)
    assert!(elapsed.as_millis() < 1000, "Unsolvable route took too long: {:?}", elapsed);
}

// ============================================================================
// Challenge 7: Intertwined Multi-Pin Nets (3-pin and 4-pin)
// ============================================================================

#[test]
fn test_challenge_intertwined_multi_pin_nets() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let config = test_config();

    // Net 1 (Triangle): (15, 15), (15, 35), (35, 25)
    board.register_net(1, "TRIANGLE");
    let t1 = Pad::new_smd("1", "T1", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let t2 = Pad::new_smd("2", "T2", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let t3 = Pad::new_smd("3", "T3", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("UT1", "P", "P", Point::new(15.0, 15.0), 0.0, vec![t1], Vec::new()));
    board.add_footprint(Footprint::new("UT2", "P", "P", Point::new(15.0, 35.0), 0.0, vec![t2], Vec::new()));
    board.add_footprint(Footprint::new("UT3", "P", "P", Point::new(35.0, 25.0), 0.0, vec![t3], Vec::new()));

    // Net 2 (Square): (20, 20), (20, 30), (30, 30), (30, 20) inside the triangle!
    board.register_net(2, "SQUARE");
    let s1 = Pad::new_smd("1", "S1", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    let s2 = Pad::new_smd("2", "S2", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    let s3 = Pad::new_smd("3", "S3", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    let s4 = Pad::new_smd("4", "S4", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("US1", "P", "P", Point::new(20.0, 20.0), 0.0, vec![s1], Vec::new()));
    board.add_footprint(Footprint::new("US2", "P", "P", Point::new(20.0, 30.0), 0.0, vec![s2], Vec::new()));
    board.add_footprint(Footprint::new("US3", "P", "P", Point::new(30.0, 30.0), 0.0, vec![s3], Vec::new()));
    board.add_footprint(Footprint::new("US4", "P", "P", Point::new(30.0, 20.0), 0.0, vec![s4], Vec::new()));

    let report = route_board_with_report(&mut board, &config).expect("Multi-pin nets must route");
    assert_eq!(report.routed_nets, 2);
    assert_eq!(report.unrouted_nets, 0);

    let drc = run_drc(&board, &config);
    assert!(drc.is_clean(), "Multi-pin nets must connect all pins cleanly without shorts");
}

// ============================================================================
// Challenge 8: Mixed Layer Routing (B.Cu, F.Cu, THT)
// ============================================================================

#[test]
fn test_challenge_mixed_layer_pads_and_transitions() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let config = test_config();

    // Net 1: B.Cu pad to B.Cu pad (vertical route on bottom layer)
    board.register_net(1, "BCU_TO_BCU");
    let p1_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::BCu).with_net(1);
    let p1_b = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::BCu).with_net(1);
    board.add_footprint(Footprint::new("U1A", "P", "P", Point::new(15.0, 15.0), 0.0, vec![p1_a], Vec::new()));
    board.add_footprint(Footprint::new("U1B", "P", "P", Point::new(15.0, 35.0), 0.0, vec![p1_b], Vec::new()));

    // Net 2: F.Cu pad to B.Cu pad (requires at least one via)
    board.register_net(2, "FCU_TO_BCU");
    let p2_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    let p2_b = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::BCu).with_net(2);
    board.add_footprint(Footprint::new("U2A", "P", "P", Point::new(20.0, 15.0), 0.0, vec![p2_a], Vec::new()));
    board.add_footprint(Footprint::new("U2B", "P", "P", Point::new(25.0, 35.0), 0.0, vec![p2_b], Vec::new()));

    // Net 3: THT pad to B.Cu pad
    board.register_net(3, "THT_TO_BCU");
    let p3_a = Pad::new_tht("1", "A", Point::new(0.0, 0.0), PadShape::Circle { diameter: 1.6 }, 0.8, 1.6).with_net(3);
    let p3_b = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::BCu).with_net(3);
    board.add_footprint(Footprint::new("U3A", "P", "P", Point::new(30.0, 15.0), 0.0, vec![p3_a], Vec::new()));
    board.add_footprint(Footprint::new("U3B", "P", "P", Point::new(30.0, 35.0), 0.0, vec![p3_b], Vec::new()));

    // Net 4: THT pad to F.Cu pad
    board.register_net(4, "THT_TO_FCU");
    let p4_a = Pad::new_tht("1", "A", Point::new(0.0, 0.0), PadShape::Circle { diameter: 1.6 }, 0.8, 1.6).with_net(4);
    let p4_b = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(4);
    board.add_footprint(Footprint::new("U4A", "P", "P", Point::new(35.0, 15.0), 0.0, vec![p4_a], Vec::new()));
    board.add_footprint(Footprint::new("U4B", "P", "P", Point::new(35.0, 35.0), 0.0, vec![p4_b], Vec::new()));

    route_board(&mut board, &config).expect("Mixed layer nets must route");

    let drc = run_drc(&board, &config);
    assert!(drc.is_clean(), "Mixed layer nets must have zero DRC violations");
}

// ============================================================================
// Challenge 9: Corner Pinching Avoidance
// ============================================================================

#[test]
fn test_challenge_corner_pinching_guard_active() {
    let mut board = PcbBoard::new(40.0, 40.0);
    let config = test_config();

    // Two foreign obstacle pads placed diagonally at (20.0, 20.0) and (21.5, 21.5)
    // The orthogonal cells between them are blocked. A naive diagonal pathfinder
    // would step between their corners, clipping clearance or shorting!
    board.register_net(1, "OBSTACLE_DIAG");
    let p_obs1 = Pad::new_smd("1", "O1", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let p_obs2 = Pad::new_smd("2", "O2", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U_O1", "P", "P", Point::new(20.0, 20.0), 0.0, vec![p_obs1], Vec::new()));
    board.add_footprint(Footprint::new("U_O2", "P", "P", Point::new(21.5, 21.5), 0.0, vec![p_obs2], Vec::new()));

    // Net 2 wants to cross from (18.0, 22.0) to (23.0, 19.0)
    board.register_net(2, "CROSS_NET");
    let p_cr1 = Pad::new_smd("1", "C1", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(2);
    let p_cr2 = Pad::new_smd("2", "C2", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U_C1", "P", "P", Point::new(18.0, 22.0), 0.0, vec![p_cr1], Vec::new()));
    board.add_footprint(Footprint::new("U_C2", "P", "P", Point::new(23.0, 19.0), 0.0, vec![p_cr2], Vec::new()));

    route_board(&mut board, &config).expect("Crossing net must route cleanly without pinching");

    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "Corner pinching avoidance must prevent shorts");
    assert_eq!(drc.clearance_violations.len(), 0, "Corner pinching avoidance must maintain clearance");
}

// ============================================================================
// Challenge 10: Performance Scaling on Multi-Net Congested Boards
// ============================================================================

#[test]
fn test_challenge_performance_scaling_sixteen_nets() {
    let mut board = PcbBoard::new(60.0, 60.0);
    let config = test_config();

    // 8 horizontal nets and 8 vertical nets creating 64 crossing intersection points
    // Horizontal nets (1..=8) at y = 12, 16, 20, 24, 28, 32, 36, 40 across x in [10, 50]
    for i in 0..8 {
        let net_id = (i + 1) as NetId;
        board.register_net(net_id, format!("H_PERF_{}", net_id));
        let y = 12.0 + (i as f64) * 4.0;
        let p1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(net_id);
        let p2 = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(net_id);
        board.add_footprint(Footprint::new(format!("UH_S{}", net_id), "P", "P", Point::new(10.0, y), 0.0, vec![p1], Vec::new()));
        board.add_footprint(Footprint::new(format!("UH_E{}", net_id), "P", "P", Point::new(50.0, y), 0.0, vec![p2], Vec::new()));
    }

    // Vertical nets (9..=16) at x = 12, 16, 20, 24, 28, 32, 36, 40 across y in [10, 50]
    for i in 0..8 {
        let net_id = (i + 9) as NetId;
        board.register_net(net_id, format!("V_PERF_{}", net_id));
        let x = 12.0 + (i as f64) * 4.0;
        let p1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(net_id);
        let p2 = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(net_id);
        board.add_footprint(Footprint::new(format!("UV_S{}", net_id), "P", "P", Point::new(x, 10.0), 0.0, vec![p1], Vec::new()));
        board.add_footprint(Footprint::new(format!("UV_E{}", net_id), "P", "P", Point::new(x, 50.0), 0.0, vec![p2], Vec::new()));
    }

    let start = Instant::now();
    let report = route_board_with_report(&mut board, &config).expect("16-net board must route successfully");
    let duration = start.elapsed();

    assert_eq!(report.routed_nets, 16);
    assert_eq!(report.unrouted_nets, 0);

    // Performance assertion: routing 16 crossing nets must complete in under 3.0 seconds
    assert!(
        duration.as_secs_f64() < 3.0,
        "Performance test failed: took {:.2}s, expected < 3.0s",
        duration.as_secs_f64()
    );

    let drc = run_drc(&board, &config);
    assert!(drc.is_clean(), "16-net board must pass DRC cleanly");
}

// ============================================================================
// Challenge 11: True Rip-Up and Reroute (RRR) Negotiated Conflict Resolution
// ============================================================================

#[test]
fn test_challenge_rrr_negotiated_conflict_multi_iteration() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let config = test_config();

    // Create a scenario where Net 1 and Net 2 compete for a restricted bottleneck.
    // Net 1 is slightly shorter, so it routes first in a straight line on F.Cu.
    // Net 2 is blocked by Net 1 on F.Cu.
    // Net 2 negotiates through Net 1's path, forcing Net 1 to rip up and reroute.
    board.register_net(1, "NET_A");
    board.register_net(2, "NET_B");

    // Net 1: (15.0, 25.0) to (35.0, 25.0)
    let p1_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let p1_b = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1A", "P", "P", Point::new(15.0, 25.0), 0.0, vec![p1_a], Vec::new()));
    board.add_footprint(Footprint::new("U1B", "P", "P", Point::new(35.0, 25.0), 0.0, vec![p1_b], Vec::new()));

    // Net 2: (25.0, 15.0) to (25.0, 35.0)
    let p2_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    let p2_b = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U2A", "P", "P", Point::new(25.0, 15.0), 0.0, vec![p2_a], Vec::new()));
    board.add_footprint(Footprint::new("U2B", "P", "P", Point::new(25.0, 35.0), 0.0, vec![p2_b], Vec::new()));

    // Place obstacle pads that restrict lateral movement around the crossing
    board.register_net(3, "WALL_LEFT");
    board.register_net(4, "WALL_RIGHT");
    let pw_l = Pad::new_smd("1", "W", Point::new(0.0, 0.0), PadShape::Rect { width: 2.0, height: 6.0 }, Layer::FCu).with_net(3);
    let pw_r = Pad::new_smd("1", "W", Point::new(0.0, 0.0), PadShape::Rect { width: 2.0, height: 6.0 }, Layer::FCu).with_net(4);
    board.add_footprint(Footprint::new("UW_L", "P", "P", Point::new(22.0, 25.0), 0.0, vec![pw_l], Vec::new()));
    board.add_footprint(Footprint::new("UW_R", "P", "P", Point::new(28.0, 25.0), 0.0, vec![pw_r], Vec::new()));

    let report = route_board_with_report(&mut board, &config).expect("Bottleneck crossing must route via RRR");
    assert_eq!(report.routed_nets, 2);
    assert_eq!(report.unrouted_nets, 0);

    let drc = run_drc(&board, &config);
    assert!(drc.is_clean(), "Bottleneck crossing must pass DRC");
}

// ============================================================================
// Challenge 12: Diagonal Routing with Circular & Obround Pads
// ============================================================================

#[test]
fn test_challenge_diagonal_routing_near_circular_and_obround_pads() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let config = test_config();

    // Net 1: Circular pad obstacle at (25.0, 25.0)
    board.register_net(1, "CIRC_PAD");
    let p_circ = Pad::new_smd("1", "C", Point::new(0.0, 0.0), PadShape::Circle { diameter: 2.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U_CIRC", "P", "P", Point::new(25.0, 25.0), 0.0, vec![p_circ], Vec::new()));

    // Net 2: Obround pad obstacle at (32.0, 25.0)
    board.register_net(2, "OBROUND_PAD");
    let p_obround = Pad::new_smd("1", "O", Point::new(0.0, 0.0), PadShape::Obround { width: 3.0, height: 1.5 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U_OBR", "P", "P", Point::new(32.0, 25.0), 45.0, vec![p_obround], Vec::new()));

    // Net 3: Diagonal route passing between them: (20.0, 20.0) to (35.0, 35.0)
    board.register_net(3, "DIAG_PASS");
    let p3_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(3);
    let p3_b = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(3);
    board.add_footprint(Footprint::new("U3A", "P", "P", Point::new(20.0, 20.0), 0.0, vec![p3_a], Vec::new()));
    board.add_footprint(Footprint::new("U3B", "P", "P", Point::new(35.0, 35.0), 0.0, vec![p3_b], Vec::new()));

    route_board(&mut board, &config).expect("Diagonal route must navigate near circular/obround pads");

    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0);
    assert_eq!(drc.clearance_violations.len(), 0);
    assert_eq!(drc.unrouted_nets.len(), 0);
}

// ============================================================================
// Challenge 13: Edge Keepout Proximity & Clearance Integrity
// ============================================================================

#[test]
fn test_challenge_near_edge_routing_and_keepout_validation() {
    let mut board = PcbBoard::new(40.0, 40.0);
    let config = test_config();

    // Place pads safely outside keepout (keepout is 0.40mm)
    // Pad center at (1.5, 5.0) and (1.5, 35.0)
    board.register_net(1, "EDGE_NET");
    let p1_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(1);
    let p1_b = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("UE1", "P", "P", Point::new(1.5, 5.0), 0.0, vec![p1_a], Vec::new()));
    board.add_footprint(Footprint::new("UE2", "P", "P", Point::new(1.5, 35.0), 0.0, vec![p1_b], Vec::new()));

    route_board(&mut board, &config).expect("Near-edge net must route");

    let drc = run_drc(&board, &config);
    assert_eq!(drc.edge_keepout_violations.len(), 0, "Near-edge route must respect board keepout");
    assert!(drc.is_clean());
}

// ============================================================================
// Challenge 14: Degenerate Segment & Octilinear Geometric Invariants
// ============================================================================

#[test]
fn test_challenge_geometric_invariants_across_all_tracks() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let config = test_config();

    // Multiple zigzagging nets
    for net_id in 1..=4 {
        board.register_net(net_id, format!("ZIG_{}", net_id));
        let x_s = 10.0 + (net_id as f64) * 2.0;
        let y_s = 10.0;
        let x_e = 40.0;
        let y_e = 20.0 + (net_id as f64) * 5.0;

        let p_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(net_id);
        let p_b = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(net_id);
        board.add_footprint(Footprint::new(format!("UZ_S{}", net_id), "P", "P", Point::new(x_s, y_s), 0.0, vec![p_a], Vec::new()));
        board.add_footprint(Footprint::new(format!("UZ_E{}", net_id), "P", "P", Point::new(x_e, y_e), 0.0, vec![p_b], Vec::new()));
    }

    route_board(&mut board, &config).expect("Zigzag nets must route");

    assert!(!board.tracks.is_empty(), "Must produce tracks");

    for (idx, track) in board.tracks.iter().enumerate() {
        // Invariant 1: No degenerate zero-length tracks
        assert!(
            track.length() > 1e-5,
            "Track #{} is zero-length: ({},{}) to ({},{})",
            idx, track.start.x, track.start.y, track.end.x, track.end.y
        );

        // Invariant 2: Track width matches configuration
        assert!(
            (track.width - config.default_trace_width).abs() < 1e-6,
            "Track #{} width {} != config {}",
            idx, track.width, config.default_trace_width
        );

        // Invariant 3: Strict 45/90 octilinear geometry
        let dx = (track.end.x - track.start.x).abs();
        let dy = (track.end.y - track.start.y).abs();
        let is_horiz = dy < 1e-4;
        let is_vert = dx < 1e-4;
        let is_diag_45 = (dx - dy).abs() < 1e-4;
        assert!(
            is_horiz || is_vert || is_diag_45,
            "Track #{} violates octilinear rule: dx={}, dy={}",
            idx, dx, dy
        );
    }
}

// ============================================================================
// Challenge 15: Mixed Netlist (Single-Pin, Multi-Pin, Unconnected)
// ============================================================================

#[test]
fn test_challenge_mixed_single_and_multi_pin_netlist() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let config = test_config();

    // Net 1: Single-pin net (testpoint)
    board.register_net(1, "TP1");
    let p_tp = Pad::new_smd("1", "TP", Point::new(0.0, 0.0), PadShape::Circle { diameter: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("TP1", "TP", "TP", Point::new(10.0, 10.0), 0.0, vec![p_tp], Vec::new()));

    // Net 2: 2-pin net
    board.register_net(2, "NET_2P");
    let p2_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(2);
    let p2_b = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U2A", "P", "P", Point::new(15.0, 15.0), 0.0, vec![p2_a], Vec::new()));
    board.add_footprint(Footprint::new("U2B", "P", "P", Point::new(35.0, 15.0), 0.0, vec![p2_b], Vec::new()));

    // Net 3: 5-pin net (power / bus line)
    board.register_net(3, "VCC_5PIN");
    for i in 0..5 {
        let x = 15.0 + (i as f64) * 5.0;
        let p = Pad::new_smd("1", "V", Point::new(0.0, 0.0), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(3);
        board.add_footprint(Footprint::new(format!("UVCC_{}", i), "P", "P", Point::new(x, 30.0), 0.0, vec![p], Vec::new()));
    }

    let report = route_board_with_report(&mut board, &config).expect("Mixed netlist must route");
    // Single pin net 1 is ignored by router; nets 2 and 3 are routed
    assert_eq!(report.routed_nets, 2);
    assert_eq!(report.unrouted_nets, 0);

    let drc = run_drc(&board, &config);
    assert!(drc.is_clean(), "Mixed netlist must pass DRC with 0 unrouted nets");
}

// ============================================================================
// Challenge 16: Annular Ring Sizing & Via Geometry Invariants
// ============================================================================

#[test]
fn test_challenge_via_annular_ring_and_drill_invariants() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let config = test_config();

    // Crossing nets forcing via insertion
    board.register_net(1, "N1");
    board.register_net(2, "N2");
    let p1_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let p1_b = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1A", "P", "P", Point::new(10.0, 25.0), 0.0, vec![p1_a], Vec::new()));
    board.add_footprint(Footprint::new("U1B", "P", "P", Point::new(40.0, 25.0), 0.0, vec![p1_b], Vec::new()));

    let p2_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    let p2_b = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U2A", "P", "P", Point::new(25.0, 10.0), 0.0, vec![p2_a], Vec::new()));
    board.add_footprint(Footprint::new("U2B", "P", "P", Point::new(25.0, 40.0), 0.0, vec![p2_b], Vec::new()));

    route_board(&mut board, &config).expect("Crossing nets must route");

    assert!(!board.vias.is_empty(), "Must produce vias");

    const MIN_ANNULAR_RING: f64 = 0.15;
    for via in &board.vias {
        assert_eq!(via.size, config.via_diameter);
        assert_eq!(via.drill, config.via_drill);
        assert!(via.drill < via.size, "Via drill must be smaller than via outer diameter");
        assert!(
            via.annular_ring() >= MIN_ANNULAR_RING - 1e-6,
            "Via annular ring {} < min {}",
            via.annular_ring(),
            MIN_ANNULAR_RING
        );
    }

    let drc = run_drc(&board, &config);
    assert_eq!(drc.annular_ring_violations.len(), 0);
    assert!(drc.is_clean());
}

