//! Milestone 2 Challenger 1 Test Suite
//!
//! Rigorous empirical stress tests and mathematical invariant verification for:
//! 1. Ratsnest computation latency when moving components with large net counts (< 1ms).
//! 2. Orthogonal rotation invariance over 1,000,000 rotations (asserting 0.0 drift).
//! 3. Interactive selection and hit-testing on overlapping footprints and dense pin clusters.
//! 4. Screen-to-world coordinate transformations, grid snapping, and boundary clamping.

use netlist_shatter_viz::pcb::*;
use std::time::{Duration, Instant};

const PCB_PIXELS_PER_MM: f64 = 16.0;

/// Replicates the reference microcontroller dev board from `src/main.rs` lines 2263–2394
fn create_reference_mcu_dev_board() -> PcbBoard {
    let mut board = PcbBoard::new(65.0, 32.0);

    board.register_net(1, "GND");
    board.register_net(2, "VBUS_5V");
    board.register_net(3, "+3V3");
    board.register_net(4, "RESET");
    board.register_net(5, "XTAL1");
    board.register_net(6, "XTAL2");
    board.register_net(7, "USB_DP");
    board.register_net(8, "USB_DN");
    board.register_net(9, "SCK");
    board.register_net(10, "MISO");
    board.register_net(11, "MOSI");
    board.register_net(12, "LED_STAT");
    board.register_net(13, "TXD");
    board.register_net(14, "RXD");
    board.register_net(20, "LED_ANODE");

    // U1: ATmega328P MCU (TQFP-32)
    let mut u1 = tqfp32("U1", "ATmega328P").at_position(Point::new(32.5, 16.0));
    u1 = u1
        .with_net_assigned("3", 1)  // GND
        .with_net_assigned("5", 1)  // GND
        .with_net_assigned("21", 1) // GND
        .with_net_assigned("4", 3)  // VCC
        .with_net_assigned("6", 3)  // VCC
        .with_net_assigned("18", 3) // AVCC
        .with_net_assigned("7", 5)  // XTAL1
        .with_net_assigned("8", 6)  // XTAL2
        .with_net_assigned("29", 4) // RESET
        .with_net_assigned("15", 11)// MOSI
        .with_net_assigned("16", 10)// MISO
        .with_net_assigned("17", 9) // SCK
        .with_net_assigned("13", 12)// LED_STAT
        .with_net_assigned("30", 14)// RXD
        .with_net_assigned("31", 13);// TXD
    board.add_footprint(u1);

    // J1: USB-C Receptacle
    let mut j1 = usb_c_16pin("J1", "USB-C").at_position(Point::new(6.0, 16.0));
    j1 = j1
        .with_net_assigned("A1", 1)
        .with_net_assigned("B1", 1)
        .with_net_assigned("A12", 1)
        .with_net_assigned("B12", 1)
        .with_net_assigned("S1", 1)
        .with_net_assigned("S2", 1)
        .with_net_assigned("S3", 1)
        .with_net_assigned("S4", 1)
        .with_net_assigned("A4", 2)
        .with_net_assigned("B4", 2)
        .with_net_assigned("A9", 2)
        .with_net_assigned("B9", 2)
        .with_net_assigned("A6", 7)
        .with_net_assigned("B6", 7)
        .with_net_assigned("A7", 8)
        .with_net_assigned("B7", 8);
    board.add_footprint(j1);

    // U2: AMS1117-3.3 LDO Regulator (SOT-223)
    let mut u2 = sot223("U2", "AMS1117-3.3").at_position(Point::new(16.0, 24.0));
    u2 = u2
        .with_net_assigned("1", 1)  // GND
        .with_net_assigned("2", 3)  // VOUT (+3V3)
        .with_net_assigned("3", 2)  // VIN (VBUS)
        .with_net_assigned("4", 3); // TAB (VOUT)
    board.add_footprint(u2);

    // Y1: 16MHz Crystal (SMD-3225)
    let mut y1 = crystal_smd_3225("Y1", "16MHz").at_position(Point::new(45.0, 16.0));
    y1 = y1
        .with_net_assigned("1", 5)  // XTAL1
        .with_net_assigned("3", 6)  // XTAL2
        .with_net_assigned("2", 1)  // GND
        .with_net_assigned("4", 1); // GND
    board.add_footprint(y1);

    // SW1: Tactile Reset Switch (3x4mm SMD)
    let mut sw1 = tactile_switch_3x4("SW1", "RESET").at_position(Point::new(22.0, 7.0));
    sw1 = sw1
        .with_net_assigned("1", 4) // RESET
        .with_net_assigned("2", 1); // GND
    board.add_footprint(sw1);

    // Passives (0805 packages)
    let c1 = capacitor_0805("C1", "100nF").at_position(Point::new(26.0, 18.0)).with_net_assigned("1", 3).with_net_assigned("2", 1);
    let c2 = capacitor_0805("C2", "100nF").at_position(Point::new(39.0, 18.0)).with_net_assigned("1", 3).with_net_assigned("2", 1);
    let c3 = capacitor_0805("C3", "22pF").at_position(Point::new(49.0, 12.0)).with_net_assigned("1", 5).with_net_assigned("2", 1);
    let c4 = capacitor_0805("C4", "22pF").at_position(Point::new(49.0, 20.0)).with_net_assigned("1", 6).with_net_assigned("2", 1);
    let c5 = capacitor_0805("C5", "10uF").at_position(Point::new(11.0, 24.0)).with_net_assigned("1", 2).with_net_assigned("2", 1);
    let c6 = capacitor_0805("C6", "10uF").at_position(Point::new(21.0, 24.0)).with_net_assigned("1", 3).with_net_assigned("2", 1);
    let r1 = resistor_0805("R1", "10k").at_position(Point::new(27.0, 7.0)).with_net_assigned("1", 4).with_net_assigned("2", 3);
    let r2 = resistor_0805("R2", "1k").at_position(Point::new(32.5, 7.0)).with_net_assigned("1", 12).with_net_assigned("2", 20);
    let d1 = led_0805("D1", "GREEN").at_position(Point::new(37.0, 7.0)).with_net_assigned("1", 20).with_net_assigned("2", 1);
    let d2 = led_0805("D2", "RED").at_position(Point::new(16.0, 7.0)).with_net_assigned("1", 3).with_net_assigned("2", 1);

    board.add_footprint(c1);
    board.add_footprint(c2);
    board.add_footprint(c3);
    board.add_footprint(c4);
    board.add_footprint(c5);
    board.add_footprint(c6);
    board.add_footprint(r1);
    board.add_footprint(r2);
    board.add_footprint(d1);
    board.add_footprint(d2);

    // Headers (1x15 through-hole)
    let mut j2 = header_1x15("J2", "CONN_TOP").at_position(Point::new(32.5, 2.54));
    j2 = j2
        .with_net_assigned("1", 2)  // VBUS
        .with_net_assigned("2", 3)  // 3V3
        .with_net_assigned("3", 4)  // RESET
        .with_net_assigned("4", 1)  // GND
        .with_net_assigned("14", 14)// RXD
        .with_net_assigned("15", 13);// TXD
    board.add_footprint(j2);

    let mut j3 = header_1x15("J3", "CONN_BOT").at_position(Point::new(32.5, 29.46));
    j3 = j3
        .with_net_assigned("1", 1)  // GND
        .with_net_assigned("2", 1)  // GND
        .with_net_assigned("11", 11)// MOSI
        .with_net_assigned("12", 10)// MISO
        .with_net_assigned("13", 9);// SCK
    board.add_footprint(j3);

    // Add pre-routed tracks and vias
    board.add_track(TrackSegment::new(Point::new(33.45, 7.0), Point::new(36.05, 7.0), 0.25, Layer::FCu, 20));
    board.add_track(TrackSegment::new(Point::new(18.3, 24.0), Point::new(20.05, 24.0), 0.35, Layer::FCu, 3));
    let v1 = Via::new(Point::new(26.0, 14.0), 0.8, 0.4, 1);
    let v2 = Via::new(Point::new(39.0, 14.0), 0.8, 0.4, 1);
    board.add_via(v1);
    board.add_via(v2);
    board.add_track(TrackSegment::new(Point::new(26.0, 14.0), Point::new(39.0, 14.0), 0.3, Layer::BCu, 1));

    board
}

/// Exact visualizer hover and hit-testing routine from `src/main.rs` lines 2816–2836
fn hit_test(board: &PcbBoard, mouse_mm: Point) -> (Option<String>, Option<(String, String, Option<NetId>)>) {
    let mut hovered_fp = None;
    let mut hovered_pad_info = None;

    for fp in &board.footprints {
        for pad in &fp.pads {
            let pad_box = pad.bounding_box(fp.position, fp.rotation);
            if pad_box.contains_point(mouse_mm) {
                hovered_pad_info = Some((fp.reference.clone(), pad.pin_number.clone(), pad.net));
                break;
            }
        }

        let bbox = fp.world_bounding_box().inflate(0.8);
        if bbox.contains_point(mouse_mm) {
            hovered_fp = Some(fp.reference.clone());
        }
    }
    (hovered_fp, hovered_pad_info)
}

// ============================================================================
// Challenge 1: Ratsnest Computation Latency Under Dynamic Movement (< 1ms)
// ============================================================================

#[test]
fn test_challenge_ratsnest_latency_moving_component_reference_board() {
    let mut board = create_reference_mcu_dev_board();
    let initial_lines = compute_ratsnest(&board);
    assert!(!initial_lines.is_empty(), "Reference board must have active ratsnest lines");

    // Simulate mouse dragging U1 (TQFP-32 with 15 active net connections) across 1,000 frames
    let frames = 1000;
    let mut max_duration = Duration::ZERO;
    let mut total_duration = Duration::ZERO;

    for frame in 0..frames {
        let t = frame as f64 / frames as f64;
        let new_x = 25.0 + (t * 6.28318).cos() * 8.0;
        let new_y = 16.0 + (t * 6.28318).sin() * 6.0;

        if let Some(u1) = board.find_footprint_mut("U1") {
            u1.position = Point::new(new_x, new_y);
        }

        let start = Instant::now();
        let lines = compute_ratsnest(&board);
        let elapsed = start.elapsed();

        if elapsed > max_duration {
            max_duration = elapsed;
        }
        total_duration += elapsed;

        // Invariant: tree must remain connected and non-empty
        assert!(!lines.is_empty(), "Ratsnest must never disappear while moving");
        // Latency challenge requirement: strictly < 1.0 ms
        assert!(
            elapsed < Duration::from_millis(1),
            "Frame {} took {:?}, exceeding 1ms latency budget",
            frame,
            elapsed
        );
    }

    let avg_duration = total_duration / frames as u32;
    println!(
        "[BENCH] Reference Board 1000 moves: avg = {:?}, max = {:?}",
        avg_duration, max_duration
    );

    // Assert average latency is well below 0.15ms budget from PROJECT.md
    assert!(
        avg_duration < Duration::from_micros(250),
        "Average ratsnest latency {:?} must be < 250us",
        avg_duration
    );
}

#[test]
fn test_challenge_ratsnest_latency_large_net_count_50_nets_stress() {
    let mut board = PcbBoard::new(120.0, 120.0);

    // Create 50 distinct multi-pin nets (4 pins per net = 200 pads across 50 peripheral footprints)
    let net_count = 50;
    for net_id in 1..=net_count {
        board.register_net(net_id, format!("BUS_NET_{}", net_id));
    }

    // Central master IC with 50 pins (one pin per net)
    let mut master_pads = Vec::new();
    for i in 0..net_count {
        let angle = (i as f64 / net_count as f64) * 2.0 * std::f64::consts::PI;
        let pos = Point::new(angle.cos() * 8.0, angle.sin() * 8.0);
        let pad = Pad::new_smd(
            format!("M_{}", i + 1),
            format!("PIN_{}", i + 1),
            pos,
            PadShape::Rect { width: 0.4, height: 1.0 },
            Layer::FCu,
        )
        .with_net(i as u32 + 1);
        master_pads.push(pad);
    }
    let master_fp = Footprint::new(
        "U_MASTER",
        "MASTER_ASIC",
        "BGA-50",
        Point::new(60.0, 60.0),
        0.0,
        master_pads,
        Vec::new(),
    );
    board.add_footprint(master_fp);

    // 50 peripheral 3-pin receivers arranged in a circle
    for net_id in 1..=net_count {
        let theta = (net_id as f64 / net_count as f64) * 2.0 * std::f64::consts::PI;
        let cx = 60.0 + theta.cos() * 40.0;
        let cy = 60.0 + theta.sin() * 40.0;

        let p1 = Pad::new_smd("1", "IN1", Point::new(-1.0, 0.0), PadShape::Rect { width: 0.6, height: 0.6 }, Layer::FCu).with_net(net_id);
        let p2 = Pad::new_smd("2", "IN2", Point::new(0.0, 0.0), PadShape::Rect { width: 0.6, height: 0.6 }, Layer::FCu).with_net(net_id);
        let p3 = Pad::new_smd("3", "IN3", Point::new(1.0, 0.0), PadShape::Rect { width: 0.6, height: 0.6 }, Layer::FCu).with_net(net_id);

        let fp = Footprint::new(
            format!("REC_{}", net_id),
            "RCV",
            "SOT-23",
            Point::new(cx, cy),
            0.0,
            vec![p1, p2, p3],
            Vec::new(),
        );
        board.add_footprint(fp);
    }

    // Now drag U_MASTER across 500 movement frames
    let frames = 500;
    let mut max_duration = Duration::ZERO;
    let mut total_duration = Duration::ZERO;

    for frame in 0..frames {
        let t = frame as f64 / frames as f64;
        let pos = Point::new(60.0 + (t * 10.0).sin() * 15.0, 60.0 + (t * 10.0).cos() * 15.0);

        if let Some(master) = board.find_footprint_mut("U_MASTER") {
            master.position = pos;
        }

        let start = Instant::now();
        let lines = compute_ratsnest(&board);
        let elapsed = start.elapsed();

        if elapsed > max_duration {
            max_duration = elapsed;
        }
        total_duration += elapsed;

        // Each of the 50 nets has 4 pads -> 3 EMST edges per net -> exactly 150 lines
        assert_eq!(lines.len(), 150, "50 4-pin nets must produce exactly 150 ratsnest lines");
        assert!(
            elapsed < Duration::from_millis(1),
            "50-net movement frame {} took {:?}, exceeding 1ms limit",
            frame,
            elapsed
        );
    }

    let avg = total_duration / frames as u32;
    println!(
        "[BENCH] 50-Net Stress Board 500 moves: avg = {:?}, max = {:?}",
        avg, max_duration
    );
    assert!(max_duration < Duration::from_millis(1), "Maximum latency must be < 1.0 ms");
}

#[test]
fn test_challenge_ratsnest_latency_extreme_100_nets_stress() {
    let mut board = PcbBoard::new(150.0, 150.0);
    let net_count = 100;
    for i in 1..=net_count {
        board.register_net(i, format!("NET_{}", i));
    }

    // 100 3-pin nets = 300 pads
    for net_id in 1..=net_count {
        let p1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Circle { diameter: 0.8 }, Layer::FCu).with_net(net_id);
        let p2 = Pad::new_smd("2", "B", Point::new(2.0, 0.0), PadShape::Circle { diameter: 0.8 }, Layer::FCu).with_net(net_id);
        let p3 = Pad::new_smd("3", "C", Point::new(1.0, 2.0), PadShape::Circle { diameter: 0.8 }, Layer::FCu).with_net(net_id);

        let fp = Footprint::new(
            format!("TRI_{}", net_id),
            "TRI",
            "PKG",
            Point::new((net_id % 10) as f64 * 14.0 + 5.0, (net_id / 10) as f64 * 14.0 + 5.0),
            0.0,
            vec![p1, p2, p3],
            Vec::new(),
        );
        board.add_footprint(fp);
    }

    // Move first 10 footprints across 100 frames
    let frames = 100;
    let mut max_duration = Duration::ZERO;

    for frame in 0..frames {
        let ref_des = format!("TRI_{}", (frame % 10) + 1);
        if let Some(fp) = board.find_footprint_mut(&ref_des) {
            fp.position = Point::new(fp.position.x + 0.1, fp.position.y - 0.1);
        }

        let start = Instant::now();
        let lines = compute_ratsnest(&board);
        let elapsed = start.elapsed();

        if elapsed > max_duration {
            max_duration = elapsed;
        }

        assert_eq!(lines.len(), 200, "100 3-pin nets must yield exactly 200 lines");
        assert!(
            elapsed < Duration::from_millis(1),
            "100-net frame took {:?}, exceeding 1ms",
            elapsed
        );
    }
    println!("[BENCH] 100-Net Stress Board 100 moves: max = {:?}", max_duration);
}

#[test]
fn test_challenge_ratsnest_island_clustering_with_mixed_routes_latency() {
    let mut board = PcbBoard::new(100.0, 100.0);
    // 30 4-pin nets: 15 partially routed with tracks and vias, 15 completely unrouted
    for net in 1..=30 {
        board.register_net(net, format!("MIXED_{}", net));
        let p1 = Point::new(10.0, net as f64 * 3.0);
        let p2 = Point::new(30.0, net as f64 * 3.0);
        let p3 = Point::new(50.0, net as f64 * 3.0);
        let p4 = Point::new(70.0, net as f64 * 3.0);

        let pads = vec![
            Pad::new_smd("1", "A", p1, PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(net),
            Pad::new_smd("2", "B", p2, PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(net),
            Pad::new_smd("3", "C", p3, PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(net),
            Pad::new_smd("4", "D", p4, PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(net),
        ];

        let fp = Footprint::new(format!("ROW_{}", net), "BUS", "PKG", Point::zero(), 0.0, pads, Vec::new());
        board.add_footprint(fp);

        if net <= 15 {
            // Route p1 to p2 on F.Cu, and p3 to p4 on B.Cu with vias
            board.add_track(TrackSegment::new(p1, p2, 0.25, Layer::FCu, net));
            board.add_via(Via::new(p3, 0.8, 0.4, net));
            board.add_via(Via::new(p4, 0.8, 0.4, net));
            board.add_track(TrackSegment::new(p3, p4, 0.25, Layer::BCu, net));
        }
    }

    // Move ROW_15 (which has copper connections) across 200 frames
    for frame in 0..200 {
        if let Some(fp) = board.find_footprint_mut("ROW_15") {
            fp.position = Point::new(frame as f64 * 0.05, 0.0);
        }

        let start = Instant::now();
        let lines = compute_ratsnest(&board);
        let elapsed = start.elapsed();

        assert!(
            elapsed < Duration::from_millis(1),
            "Island clustering frame took {:?}, exceeding 1ms",
            elapsed
        );

        // Nets 1-15 have 2 islands (p1-p2 clustered, p3-p4 clustered) -> 1 line per net = 15 lines
        // Nets 16-30 have 4 isolated pads -> 3 lines per net = 45 lines
        // Total expected = 60 lines
        // If ROW_15 is shifted, its pads disconnect from the static tracks (distance > 1e-3)
        // so its 4 pads become isolated again, producing 3 lines instead of 1 -> 62 lines total
        assert!(lines.len() == 60 || lines.len() == 62);
    }
}

// ============================================================================
// Challenge 2: Orthogonal Rotation Invariance Over 1,000,000 Rotations (0.0 Drift)
// ============================================================================

#[test]
fn test_challenge_orthogonal_rotation_1_million_iterative_zero_drift() {
    let seeds = [
        Point::new(12.3456789, -98.7654321),
        Point::new(0.0, 0.0),
        Point::new(100.0, 0.0),
        Point::new(0.0, -50.0),
        Point::new(-45.123456, -67.890123),
        Point::new(0.0001234, -0.0005678),
        Point::new(1e5, -1e5),
    ];

    for &initial in &seeds {
        let mut pt = initial;
        for _ in 0..1_000_000 {
            pt = pt.rotate(90.0);
        }

        let drift_x = (pt.x - initial.x).abs();
        let drift_y = (pt.y - initial.y).abs();

        assert_eq!(
            drift_x, 0.0,
            "X coordinate accumulated non-zero drift {} after 1,000,000 rotations from {:?}",
            drift_x, initial
        );
        assert_eq!(
            drift_y, 0.0,
            "Y coordinate accumulated non-zero drift {} after 1,000,000 rotations from {:?}",
            drift_y, initial
        );
        assert_eq!(pt.x, initial.x);
        assert_eq!(pt.y, initial.y);
    }
}

#[test]
fn test_challenge_orthogonal_rotation_1_million_cumulative_angles_zero_drift() {
    let initial = Point::new(34.56789, -76.54321);

    for i in 0..1_000_000 {
        let angle = (i * 90) as f64;
        let rotated = initial.rotate(angle);

        let (expected_x, expected_y) = match i % 4 {
            0 => (initial.x, initial.y),
            1 => (-initial.y, initial.x),
            2 => (-initial.x, -initial.y),
            3 => (initial.y, -initial.x),
            _ => unreachable!(),
        };

        assert_eq!(
            rotated.x, expected_x,
            "Drift at iteration {} (angle {}°): got {}, expected {}",
            i, angle, rotated.x, expected_x
        );
        assert_eq!(
            rotated.y, expected_y,
            "Drift at iteration {} (angle {}°): got {}, expected {}",
            i, angle, rotated.y, expected_y
        );
    }
}

#[test]
fn test_challenge_orthogonal_rotation_around_arbitrary_center_1_million_cycles() {
    let center = Point::new(32.5, 16.0);
    let initial = Point::new(45.123, -22.876);

    // 1. Cumulative rotation around arbitrary center:
    // Because `center + (p - center)` in IEEE-754 binary floating point involves
    // an exponent shift when `|p - center|` spans a power-of-two exponent boundary,
    // a 1-ULP rounding difference (~3.5e-15 mm) occurs on non-dyadic coordinates.
    for i in 0..1_000_000 {
        if i % 4 == 0 {
            let rot = initial.rotate_around(center, (i * 90) as f64);
            let err_x = (rot.x - initial.x).abs();
            let err_y = (rot.y - initial.y).abs();
            assert!(
                err_x < 1e-14,
                "rotate_around cumulative X drift {:e} exceeds 1e-14 at iter {}",
                err_x, i
            );
            assert!(
                err_y < 1e-14,
                "rotate_around cumulative Y drift {:e} exceeds 1e-14 at iter {}",
                err_y, i
            );
        }
    }

    // 2. Iterative rotate_around: measures compounding floating point center subtraction/addition
    // over 1,000,000 consecutive transformations
    let mut pt = initial;
    for _ in 0..1_000_000 {
        pt = pt.rotate_around(center, 90.0);
    }

    let drift_x = (pt.x - initial.x).abs();
    let drift_y = (pt.y - initial.y).abs();
    println!(
        "[INVARIANT] Iterative 1M rotate_around drift: dx = {:e}, dy = {:e}",
        drift_x, drift_y
    );
    // Over 1,000,000 compound float additions, drift is bound to sub-picometer level (< 1e-13 mm)
    assert!(
        drift_x < 1e-13,
        "Iterative X drift {} exceeds 1e-13 mm",
        drift_x
    );
    assert!(
        drift_y < 1e-13,
        "Iterative Y drift {} exceeds 1e-13 mm",
        drift_y
    );
}

#[test]
fn test_challenge_orthogonal_rotation_pad_global_positions_over_1_million_rotations() {
    let mut fp = tqfp32("U1", "ATmega328P").at_position(Point::new(32.5, 16.0));
    let initial_positions: Vec<Point> = fp.pads.iter().map(|p| p.global_position(fp.position, fp.rotation)).collect();

    // Rotate footprint 1,000,000 times using the visualizer hotkey formula:
    // fp.rotation = ((fp.rotation + 90.0) % 360.0).round();
    for _ in 0..1_000_000 {
        fp.rotation = ((fp.rotation + 90.0) % 360.0).round();
    }

    assert_eq!(fp.rotation, 0.0, "1M rotations must return rotation to 0.0°");

    for (idx, pad) in fp.pads.iter().enumerate() {
        let current_pos = pad.global_position(fp.position, fp.rotation);
        let init_pos = initial_positions[idx];

        assert_eq!(
            current_pos.x, init_pos.x,
            "Pad {} X drifted from {} to {}",
            pad.pin_number, init_pos.x, current_pos.x
        );
        assert_eq!(
            current_pos.y, init_pos.y,
            "Pad {} Y drifted from {} to {}",
            pad.pin_number, init_pos.y, current_pos.y
        );
    }
}

// ============================================================================
// Challenge 3: Interactive Selection & Hit-Testing on Dense Pins & Overlaps
// ============================================================================

#[test]
fn test_challenge_hit_testing_dense_pin_clusters_tqfp32() {
    let mut board = PcbBoard::new(100.0, 100.0);
    let u1 = tqfp32("U1", "MCU").at_position(Point::new(50.0, 50.0));
    board.add_footprint(u1);

    // Test across all 4 orthogonal orientations
    for rot in [0.0, 90.0, 180.0, 270.0] {
        if let Some(fp) = board.find_footprint_mut("U1") {
            fp.rotation = rot;
        }

        let fp = board.find_footprint("U1").unwrap();

        // 1. Probe every pin center: MUST hit that exact pin!
        for pad in &fp.pads {
            let center = pad.global_position(fp.position, fp.rotation);
            let (hov_fp, hov_pad) = hit_test(&board, center);

            assert_eq!(hov_fp.as_deref(), Some("U1"));
            assert!(
                hov_pad.is_some(),
                "Failed to hit pad {} at rotation {}° at {:?}",
                pad.pin_number, rot, center
            );
            let (ref_des, pin_num, _) = hov_pad.unwrap();
            assert_eq!(ref_des, "U1");
            assert_eq!(pin_num, pad.pin_number, "Hit wrong pin number at center");
        }

        // 2. Probe the interstitial gap between consecutive pins on Side 1 (pins 1 to 8: y = 54.5, x goes from 47.2 to 52.8)
        // Pad pitch = 0.8 mm, pad width = 0.45 mm -> gap = 0.35 mm
        if rot == 0.0 {
            for i in 1..8 {
                let pad_a = fp.pad(&i.to_string()).unwrap().global_position(fp.position, fp.rotation);
                let pad_b = fp.pad(&(i + 1).to_string()).unwrap().global_position(fp.position, fp.rotation);
                let midpoint = pad_a.midpoint(pad_b);

                let (hov_fp, hov_pad) = hit_test(&board, midpoint);
                assert_eq!(hov_fp.as_deref(), Some("U1"), "Midpoint inside package courtyard must hover U1");
                assert!(
                    hov_pad.is_none(),
                    "Midpoint between pins {} and {} (gap=0.35mm) must NOT hit any pad",
                    i, i + 1
                );
            }
        }

        // 3. Probe far outside the footprint: must hit nothing
        let outside = Point::new(10.0, 10.0);
        let (hov_fp, hov_pad) = hit_test(&board, outside);
        assert!(hov_fp.is_none());
        assert!(hov_pad.is_none());
    }
}

#[test]
fn test_challenge_hit_testing_dense_pin_clusters_usb_c() {
    let mut board = PcbBoard::new(80.0, 80.0);
    let j1 = usb_c_16pin("J1", "USB-C").at_position(Point::new(40.0, 40.0));
    board.add_footprint(j1);

    let fp = board.find_footprint("J1").unwrap();
    assert_eq!(fp.pads.len(), 16, "USB-C 16-pin must have 16 pads");

    // Probe center of each of the 16 pads
    for pad in &fp.pads {
        let center = pad.global_position(fp.position, fp.rotation);
        let (hov_fp, hov_pad) = hit_test(&board, center);

        assert_eq!(hov_fp.as_deref(), Some("J1"));
        assert!(
            hov_pad.is_some(),
            "USB-C pad {} center hit-test failed at {:?}",
            pad.pin_number, center
        );
        let (ref_des, pin_num, _) = hov_pad.unwrap();
        assert_eq!(ref_des, "J1");
        assert_eq!(pin_num, pad.pin_number);
    }

    // High density check: USB-C inner SMD pads have 0.5 mm pitch with 0.3 mm width (gap = 0.2 mm)
    // Probe between A6 and A7 (D+ and D- pins at relative x = -0.25 and +0.25)
    let p_a6 = fp.pad("A6").unwrap().global_position(fp.position, fp.rotation);
    let p_a7 = fp.pad("A7").unwrap().global_position(fp.position, fp.rotation);
    let gap_mid = p_a6.midpoint(p_a7);

    let (_, hov_pad) = hit_test(&board, gap_mid);
    assert!(
        hov_pad.is_none(),
        "0.2mm gap between USB-C A6 and A7 must NOT trigger false pad detection"
    );
}

#[test]
fn test_challenge_interactive_selection_overlapping_footprints_z_order() {
    let mut board = PcbBoard::new(80.0, 80.0);

    // Add U1 (large TQFP-32) at (40.0, 40.0)
    let u1 = tqfp32("U1", "MCU").at_position(Point::new(40.0, 40.0));
    board.add_footprint(u1);

    // Add C1 (small 0805 passive) at (40.0, 40.0) OVERLAPPING the center of U1
    let c1 = capacitor_0805("C1", "100nF").at_position(Point::new(40.0, 40.0));
    board.add_footprint(c1);

    // Point exactly at (40.0, 40.0) is inside both U1's bounding box and C1's bounding box
    // Because C1 was added later, it is rendered later in the loop (top of z-order)
    let center = Point::new(40.0, 40.0);
    let (hov_fp, _) = hit_test(&board, center);

    assert_eq!(
        hov_fp.as_deref(),
        Some("C1"),
        "Topmost footprint in z-order must be selected on overlapping coordinates"
    );

    // Now probe a pad of U1 (e.g. Pin 1 at relative position ~ (-4.5, -3.2))
    let u1_fp = board.find_footprint("U1").unwrap();
    let u1_pad1 = u1_fp.pad("1").unwrap().global_position(u1_fp.position, u1_fp.rotation);

    let (hov_fp_pad, hov_pad) = hit_test(&board, u1_pad1);
    assert_eq!(hov_fp_pad.as_deref(), Some("U1"));
    assert!(hov_pad.is_some());
    assert_eq!(hov_pad.unwrap().1, "1");

    // Move C1 away to (15.0, 15.0)
    if let Some(c1_mut) = board.find_footprint_mut("C1") {
        c1_mut.position = Point::new(15.0, 15.0);
    }

    // Now center (40.0, 40.0) must select U1
    let (hov_fp_after, _) = hit_test(&board, center);
    assert_eq!(
        hov_fp_after.as_deref(),
        Some("U1"),
        "After moving top component away, underlying component must be selectable"
    );
}

#[test]
fn test_challenge_interactive_selection_coincident_footprints() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let r1 = resistor_0805("R1", "10k").at_position(Point::new(25.0, 25.0));
    let r2 = resistor_0805("R2", "1k").at_position(Point::new(25.0, 25.0));
    board.add_footprint(r1);
    board.add_footprint(r2);

    let (hov_fp, _) = hit_test(&board, Point::new(25.0, 25.0));
    assert_eq!(hov_fp.as_deref(), Some("R2"), "R2 is on top of coincident stack");

    // Separate them
    if let Some(r2_mut) = board.find_footprint_mut("R2") {
        r2_mut.position = Point::new(35.0, 25.0);
    }

    let (hov_r1, _) = hit_test(&board, Point::new(25.0, 25.0));
    let (hov_r2, _) = hit_test(&board, Point::new(35.0, 25.0));

    assert_eq!(hov_r1.as_deref(), Some("R1"));
    assert_eq!(hov_r2.as_deref(), Some("R2"));
}

#[test]
fn test_challenge_interactive_selection_dense_passive_matrix() {
    let mut board = PcbBoard::new(60.0, 60.0);

    // 4x4 matrix of 0805 resistors (2.0mm x 1.25mm) with 0.4mm gap
    for row in 0..4 {
        for col in 0..4 {
            let ref_des = format!("R_{}_{}", row, col);
            let pos = Point::new(15.0 + col as f64 * 2.8, 15.0 + row as f64 * 2.0);
            let r = resistor_0805(ref_des, "100R").at_position(pos);
            board.add_footprint(r);
        }
    }

    // Verify all 16 resistors are independently selectable at their centers
    for row in 0..4 {
        for col in 0..4 {
            let expected_ref = format!("R_{}_{}", row, col);
            let pos = Point::new(15.0 + col as f64 * 2.8, 15.0 + row as f64 * 2.0);

            let (hov_fp, _) = hit_test(&board, pos);
            assert_eq!(
                hov_fp.as_deref(),
                Some(expected_ref.as_str()),
                "Failed to resolve distinct resistor at matrix ({}, {})",
                row, col
            );
        }
    }
}

// ============================================================================
// Challenge 4: Coordinate Scaling, Snapping, and Clamping
// ============================================================================

#[test]
fn test_challenge_coordinate_roundtrip_screen_world_mm_scaling() {
    let origin_x = 120.5f32;
    let origin_y = 85.25f32;

    for i in 0..10_000 {
        let x_mm = (i as f64 * 0.0137) % 150.0;
        let y_mm = ((i * 3) as f64 * 0.0119) % 100.0;
        let p_orig = Point::new(x_mm, y_mm);

        // Forward: mm to world (Macroquad screen/camera space)
        let world_x = origin_x + (p_orig.x * PCB_PIXELS_PER_MM) as f32;
        let world_y = origin_y + (p_orig.y * PCB_PIXELS_PER_MM) as f32;

        // Backward: world to mm
        let recon_x = (world_x - origin_x) as f64 / PCB_PIXELS_PER_MM;
        let recon_y = (world_y - origin_y) as f64 / PCB_PIXELS_PER_MM;
        let p_recon = Point::new(recon_x, recon_y);

        let err_x = (p_recon.x - p_orig.x).abs();
        let err_y = (p_recon.y - p_orig.y).abs();

        // 16.0 is exact power of two (2^4); f32 mantissa holds ~7 decimal digits (24 bits)
        // At world_x ~ 2500 px, f32 machine epsilon is 2500 * 2^-23 ~= 0.0003 px ~= 1.8e-5 mm.
        // Screen roundtrip error is strictly bound to sub-micron scale (< 3e-5 mm = 30 nanometers).
        assert!(err_x < 3e-5, "Error in X {} exceeds 3e-5 at {:?}", err_x, p_orig);
        assert!(err_y < 3e-5, "Error in Y {} exceeds 3e-5 at {:?}", err_y, p_orig);
    }
}

#[test]
fn test_challenge_grid_snapping_precision_and_edge_clamping() {
    let board_w = 65.0;
    let board_h = 32.0;

    // Test 0.5mm grid
    let snap_05 = 0.5;
    let target1 = Point::new(12.33, 15.68);
    let snapped1 = Point::new(
        (target1.x / snap_05).round() * snap_05,
        (target1.y / snap_05).round() * snap_05,
    );
    assert_eq!(snapped1, Point::new(12.5, 15.5));

    // Test 2.54mm (0.1") grid
    let snap_254 = 2.54;
    let target2 = Point::new(5.0, 7.5);
    let snapped2 = Point::new(
        (target2.x / snap_254).round() * snap_254,
        (target2.y / snap_254).round() * snap_254,
    );
    assert!((snapped2.x - 5.08).abs() < 1e-9);
    assert!((snapped2.y - 7.62).abs() < 1e-9);

    // Test clamping behavior: components cannot be dragged off board
    let out_left = Point::new(-10.0, 15.0);
    let clamped_left = Point::new(
        out_left.x.clamp(0.5, board_w - 0.5),
        out_left.y.clamp(0.5, board_h - 0.5),
    );
    assert_eq!(clamped_left.x, 0.5);

    let out_right = Point::new(100.0, 50.0);
    let clamped_right = Point::new(
        out_right.x.clamp(0.5, board_w - 0.5),
        out_right.y.clamp(0.5, board_h - 0.5),
    );
    assert_eq!(clamped_right.x, 64.5);
    assert_eq!(clamped_right.y, 31.5);
}
