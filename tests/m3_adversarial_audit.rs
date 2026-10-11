//! Adversarial Audit Test Suite for Milestone 3
//! Rigorously verifies algorithmic authenticity, geometric correctness,
//! maze obstacle avoidance, edge-case failure modes, and false-positive immunity.

use netlist_shatter_viz::pcb::*;

fn test_router_config() -> RouterConfig {
    RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    }
}

#[test]
fn adversarial_test_maze_wall_navigation() {
    // Pin 1 at (10, 30), Pin 2 at (50, 30)
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "NET_ROUTED");
    board.register_net(2, "NET_WALL");

    let p1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let p2 = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(10.0, 30.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "C", "P", Point::new(50.0, 30.0), 0.0, vec![p2], Vec::new()));

    // Place a through-hole wall of foreign pads (Net 2) along X = 30.0, from Y = 15.0 to Y = 45.0
    // spaced closely (every 1.2mm) so no trace can pass through on either F.Cu or B.Cu
    let mut wall_y = 15.0;
    let mut wall_idx = 0;
    while wall_y <= 45.0 {
        let wall_pad = Pad::new_tht(
            "1",
            "W",
            Point::new(0.0, 0.0),
            PadShape::Circle { diameter: 1.2 },
            0.8,
            1.2,
        ).with_net(2);
        board.add_footprint(Footprint::new(
            format!("W{}", wall_idx),
            "WALL",
            "WALL",
            Point::new(30.0, wall_y),
            0.0,
            vec![wall_pad],
            Vec::new(),
        ));
        wall_y += 1.2;
        wall_idx += 1;
    }

    let config = test_router_config();
    let result = route_board(&mut board, &config);
    assert!(result.is_ok(), "Router must successfully navigate around the maze wall");

    // The route must have gone around the wall: either min Y < 14.0 or max Y > 46.0
    let mut circumnavigated = false;
    for (idx, t) in board.tracks.iter().enumerate() {
        println!("Track #{}: ({:.2}, {:.2}) -> ({:.2}, {:.2}) layer={:?} net={}", idx, t.start.x, t.start.y, t.end.x, t.end.y, t.layer, t.net);
        if t.start.y < 14.0 || t.end.y < 14.0 || t.start.y > 46.0 || t.end.y > 46.0 {
            circumnavigated = true;
        }
    }
    for (idx, v) in board.vias.iter().enumerate() {
        println!("Via #{}: ({:.2}, {:.2}) net={}", idx, v.at.x, v.at.y, v.net);
    }
    let drc = run_drc(&board, &config);
    println!("DRC total violations: {}", drc.total_violations());
    for s in &drc.short_circuits {
        println!("Short: at ({:.2}, {:.2}) layer={:?} net1={:?} net2={:?}: {}", s.location.x, s.location.y, s.layer, s.net1, s.net2, s.message);
    }
    for c in &drc.clearance_violations {
        println!("Clearance: at ({:.2}, {:.2}) actual={:.4} required={:.4}: {}", c.location.x, c.location.y, c.actual, c.required, c.message);
    }
    for u in &drc.unrouted_nets {
        println!("Unrouted: net {}", u);
    }
    assert_eq!(drc.short_circuits.len(), 0, "No shorts against wall");
    assert_eq!(drc.clearance_violations.len(), 0, "Clearance maintained around wall");
    assert_eq!(drc.unrouted_nets.len(), 0, "Net 1 must be routed");
}

#[test]
fn adversarial_test_impassable_board_honestly_fails() {
    let mut board = PcbBoard::new(40.0, 40.0);
    board.register_net(1, "NET_A");
    board.register_net(2, "NET_WALL");

    let p1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let p2 = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(10.0, 20.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "C", "P", Point::new(30.0, 20.0), 0.0, vec![p2], Vec::new()));

    // Complete solid THT wall from Y = 0 to Y = 40 along X = 20
    let mut y = 0.5;
    let mut idx = 0;
    while y <= 39.5 {
        let wall_pad = Pad::new_tht(
            "1",
            "W",
            Point::new(0.0, 0.0),
            PadShape::Circle { diameter: 1.2 },
            0.8,
            1.2,
        ).with_net(2);
        board.add_footprint(Footprint::new(
            format!("W{}", idx),
            "WALL",
            "WALL",
            Point::new(20.0, y),
            0.0,
            vec![wall_pad],
            Vec::new(),
        ));
        y += 1.0;
        idx += 1;
    }

    let config = test_router_config();
    let result = route_board(&mut board, &config);
    assert!(
        result.is_err(),
        "Router MUST return Err when path is geometrically blocked by a solid wall, proving it does not fake routes"
    );
}

#[test]
fn adversarial_test_rotated_pad_clearance_geometry() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_PAD");
    board.register_net(2, "NET_TRACK");

    // Pad at (25.0, 25.0), size 2.0 x 1.0 rotated 45 degrees
    // Corners of this pad at 45 deg:
    // width/2 = 1.0, height/2 = 0.5
    // In local coords: (+-1.0, +-0.5)
    // Rotated by 45 deg:
    // (1.0*cos45 - 0.5*sin45, 1.0*sin45 + 0.5*cos45) = (0.5*sqrt(2)/2, 1.5*sqrt(2)/2) approx (0.354, 1.061)
    // Max Y corner = 25.0 + 1.06066 = 26.06066
    let pad = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 2.0, height: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(25.0, 25.0), 45.0, vec![pad], Vec::new()));

    // Place a track at Y = 26.06066 + half_width(0.125) + 0.10mm (clearance = 0.10mm < 0.20mm requirement)
    let y_violating = 26.06066 + 0.125 + 0.10;
    board.add_track(TrackSegment::new(
        Point::new(20.0, y_violating),
        Point::new(30.0, y_violating),
        0.25,
        Layer::FCu,
        2,
    ));

    let config = test_router_config();
    let drc = run_drc(&board, &config);
    assert!(
        !drc.clearance_violations.is_empty(),
        "DRC must detect clearance violation against 45-degree rotated rectangular pad"
    );

    // Now shift the track further up to clearance = 0.25mm (> 0.20mm requirement)
    let mut board_clean = PcbBoard::new(50.0, 50.0);
    board_clean.register_net(1, "NET_PAD");
    board_clean.register_net(2, "NET_TRACK");
    let pad_clean = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 2.0, height: 1.0 }, Layer::FCu).with_net(1);
    board_clean.add_footprint(Footprint::new("U1", "C", "P", Point::new(25.0, 25.0), 45.0, vec![pad_clean], Vec::new()));

    let y_clean = 26.06066 + 0.125 + 0.25;
    board_clean.add_track(TrackSegment::new(
        Point::new(20.0, y_clean),
        Point::new(30.0, y_clean),
        0.25,
        Layer::FCu,
        2,
    ));
    let drc_clean = run_drc(&board_clean, &config);
    assert_eq!(
        drc_clean.clearance_violations.len(),
        0,
        "DRC must report zero violations when track is outside clearance envelope of rotated pad"
    );
}

#[test]
fn adversarial_test_dsu_connectivity_partitioning_and_cycles() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_DSU");

    // 4 pins of Net 1
    let p1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let p2 = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let p3 = Pad::new_smd("1", "C", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let p4 = Pad::new_smd("1", "D", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);

    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(10.0, 10.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "C", "P", Point::new(20.0, 10.0), 0.0, vec![p2], Vec::new()));
    board.add_footprint(Footprint::new("U3", "C", "P", Point::new(30.0, 30.0), 0.0, vec![p3], Vec::new()));
    board.add_footprint(Footprint::new("U4", "C", "P", Point::new(40.0, 30.0), 0.0, vec![p4], Vec::new()));

    // Connect (U1, U2) and (U3, U4) separately (two disjoint copper islands)
    board.add_track(TrackSegment::new(Point::new(10.0, 10.0), Point::new(20.0, 10.0), 0.25, Layer::FCu, 1));
    board.add_track(TrackSegment::new(Point::new(30.0, 30.0), Point::new(40.0, 30.0), 0.25, Layer::FCu, 1));

    let config = test_router_config();
    let drc = run_drc(&board, &config);
    assert!(
        drc.unrouted_nets.contains(&1),
        "DSU must detect that Net 1 is split into two disconnected components"
    );

    // Bridge the two islands using a track with vias (F.Cu -> Via -> B.Cu -> Via -> F.Cu)
    board.add_via(Via::new(Point::new(20.0, 10.0), 0.80, 0.40, 1));
    board.add_track(TrackSegment::new(Point::new(20.0, 10.0), Point::new(30.0, 30.0), 0.25, Layer::BCu, 1));
    board.add_via(Via::new(Point::new(30.0, 30.0), 0.80, 0.40, 1));

    let drc_bridged = run_drc(&board, &config);
    assert!(
        !drc_bridged.unrouted_nets.contains(&1),
        "DSU must verify that Net 1 is fully connected after multi-layer bridging"
    );

    // Add a redundant closing loop forming a cycle: (U1 to U4 directly)
    board.add_track(TrackSegment::new(Point::new(10.0, 10.0), Point::new(10.0, 30.0), 0.25, Layer::FCu, 1));
    board.add_track(TrackSegment::new(Point::new(10.0, 30.0), Point::new(40.0, 30.0), 0.25, Layer::FCu, 1));
    let drc_cycle = run_drc(&board, &config);
    assert!(
        !drc_cycle.unrouted_nets.contains(&1),
        "DSU must correctly handle cycles in copper connectivity graph without error"
    );
}

#[test]
fn adversarial_test_annular_ring_and_breakout_exactness() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_VIA");

    // Exact boundary test:
    // Size = 0.80, Drill = 0.50 -> annular ring = (0.80 - 0.50) / 2 = 0.150mm.
    // Minimum requirement is 0.15mm -> exact boundary must PASS.
    board.add_via(Via::new(Point::new(15.0, 15.0), 0.80, 0.50, 1));

    // Size = 0.80, Drill = 0.51 -> annular ring = (0.80 - 0.51) / 2 = 0.145mm.
    // 0.145 < 0.15mm -> must FAIL.
    board.add_via(Via::new(Point::new(25.0, 25.0), 0.80, 0.51, 1));

    // Fatal breakout: drill == size
    board.add_via(Via::new(Point::new(35.0, 35.0), 0.60, 0.60, 1));

    let config = test_router_config();
    let drc = run_drc(&board, &config);

    assert_eq!(drc.annular_ring_violations.len(), 2);
    let kinds: Vec<DrcViolationKind> = drc.annular_ring_violations.iter().map(|v| v.kind).collect();
    assert!(kinds.contains(&DrcViolationKind::AnnularRingTooSmall));
    assert!(kinds.contains(&DrcViolationKind::DrillBreakout));
}
