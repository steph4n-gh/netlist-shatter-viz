//! Milestone 1 Adversarial Probe Suite: Empirical Precision & Stress Testing
//!
//! Author: Milestone 1 Challenger 1 (m1_challenger_1)
//!
//! Rigorously challenges:
//! 1. Trigonometric rotation snapping (`sin_cos_deg`) across multi-turn, negative, and near-orthogonal angles
//! 2. Zero-drift guarantee over 1,000,000 rotations (250,000 full revolutions)
//! 3. Rotated PadShape bounding boxes (Rect, Obround horizontal, Obround vertical, Obround circular) vs analytical oracles
//! 4. Vector arithmetic, zero-length segment projections, zero-divisions
//! 5. EdgeCuts polygon area, non-convex L-shaped ray-casting containment, and distance calculations
//! 6. Footprint parametric geometries, pitch tolerances, and annular ring requirements
//! 7. Netlist ingestion conflict semantics and board validation error detection

use netlist_shatter_viz::pcb::*;
use std::f64::consts::PI;

// ============================================================================
// 1. Trigonometric Snapping & Extreme Angles
// ============================================================================

#[test]
fn probe_sin_cos_deg_multi_turn_orthogonals() {
    let test_turns = [
        0, 1, 2, 3, 4, 5, 10, 50, 100, 360, 1000, -1, -2, -3, -4, -5, -10, -50, -100, -360, -1000,
    ];

    for &turns in &test_turns {
        let base = (turns as f64) * 360.0;

        // 0 deg
        let (s0, c0) = sin_cos_deg(base);
        assert_eq!(s0, 0.0, "sin(0 + {} turns) must be exactly 0.0", turns);
        assert_eq!(c0, 1.0, "cos(0 + {} turns) must be exactly 1.0", turns);

        // 90 deg
        let (s90, c90) = sin_cos_deg(base + 90.0);
        assert_eq!(s90, 1.0, "sin(90 + {} turns) must be exactly 1.0", turns);
        assert_eq!(c90, 0.0, "cos(90 + {} turns) must be exactly 0.0", turns);

        // 180 deg
        let (s180, c180) = sin_cos_deg(base + 180.0);
        assert_eq!(s180, 0.0, "sin(180 + {} turns) must be exactly 0.0", turns);
        assert_eq!(c180, -1.0, "cos(180 + {} turns) must be exactly -1.0", turns);

        // 270 deg
        let (s270, c270) = sin_cos_deg(base + 270.0);
        assert_eq!(s270, -1.0, "sin(270 + {} turns) must be exactly -1.0", turns);
        assert_eq!(c270, 0.0, "cos(270 + {} turns) must be exactly 0.0", turns);
    }
}

#[test]
fn probe_sin_cos_deg_snapping_thresholds() {
    // Exactly at threshold boundary (1e-9 tolerance)
    let (s_snapped_hi, c_snapped_hi) = sin_cos_deg(90.0 + 5e-10);
    assert_eq!(s_snapped_hi, 1.0);
    assert_eq!(c_snapped_hi, 0.0);

    let (s_snapped_lo, c_snapped_lo) = sin_cos_deg(90.0 - 5e-10);
    assert_eq!(s_snapped_lo, 1.0);
    assert_eq!(c_snapped_lo, 0.0);

    // Negative 0.0
    let (s_neg_zero, c_neg_zero) = sin_cos_deg(-0.0);
    assert_eq!(s_neg_zero, 0.0);
    assert_eq!(c_neg_zero, 1.0);

    // 360 - 5e-10
    let (s_360, c_360) = sin_cos_deg(360.0 - 5e-10);
    assert_eq!(s_360, 0.0);
    assert_eq!(c_360, 1.0);
}

#[test]
fn probe_sin_cos_deg_trig_identity() {
    // For 3600 arbitrary fractional angles, sin^2 + cos^2 == 1.0
    for i in 0..3600 {
        let angle = (i as f64) * 0.1 - 180.0;
        let (s, c) = sin_cos_deg(angle);
        let id = s * s + c * c;
        assert!(
            (id - 1.0).abs() < 1e-12,
            "Trig identity failed for angle {}: sin^2 + cos^2 = {}",
            angle,
            id
        );
    }
}

#[test]
fn probe_point_one_million_rotations_zero_drift() {
    let initial = Point::new(12.3456789012345, -98.7654321098765);
    let mut p = initial;

    // 1,000,000 steps of 90 degrees = 250,000 full revolutions
    for _ in 0..1_000_000 {
        p = p.rotate(90.0);
    }

    assert_eq!(
        p.x, initial.x,
        "Bitwise exact X required after 1M rotations. Initial: {}, Final: {}",
        initial.x, p.x
    );
    assert_eq!(
        p.y, initial.y,
        "Bitwise exact Y required after 1M rotations. Initial: {}, Final: {}",
        initial.y, p.y
    );
}

// ============================================================================
// 2. Rotated Pad Bounding Box Correctness vs Analytical & Numerical Oracles
// ============================================================================

#[test]
fn probe_rotated_rect_pad_bounding_box_oracle() {
    // Challenge Rect bounding box against the analytical 4-corner envelope oracle
    let width = 5.3;
    let height = 2.7;
    let shape = PadShape::Rect { width, height };
    let center = Point::new(14.5, -23.8);

    for angle_step in 0..360 {
        let angle = angle_step as f64;
        let bbox = shape.bounding_box(center, angle);

        // Ground truth: compute 4 corners rotated around center
        let (sin_a, cos_a) = sin_cos_deg(angle);
        let hw = width / 2.0;
        let hh = height / 2.0;
        let local_corners = [
            Point::new(-hw, -hh),
            Point::new(hw, -hh),
            Point::new(hw, hh),
            Point::new(-hw, hh),
        ];

        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;

        for c in &local_corners {
            let rx = center.x + c.x * cos_a - c.y * sin_a;
            let ry = center.y + c.x * sin_a + c.y * cos_a;
            min_x = min_x.min(rx);
            max_x = max_x.max(rx);
            min_y = min_y.min(ry);
            max_y = max_y.max(ry);
        }

        assert!(
            (bbox.min.x - min_x).abs() < 1e-12,
            "Rect bbox min.x mismatch at {} deg: got {}, expected {}",
            angle,
            bbox.min.x,
            min_x
        );
        assert!(
            (bbox.max.x - max_x).abs() < 1e-12,
            "Rect bbox max.x mismatch at {} deg: got {}, expected {}",
            angle,
            bbox.max.x,
            max_x
        );
        assert!(
            (bbox.min.y - min_y).abs() < 1e-12,
            "Rect bbox min.y mismatch at {} deg: got {}, expected {}",
            angle,
            bbox.min.y,
            min_y
        );
        assert!(
            (bbox.max.y - max_y).abs() < 1e-12,
            "Rect bbox max.y mismatch at {} deg: got {}, expected {}",
            angle,
            bbox.max.y,
            max_y
        );
    }
}

#[test]
fn probe_rotated_obround_pad_bounding_box_oracle() {
    // Challenge Obround for both width >= height and width < height against
    // dense numerical sampling of boundary points (Minkowski sum verification)
    let test_cases = [
        (6.0, 2.0, "horizontal"), // width >= height
        (2.0, 6.0, "vertical"),   // width < height
        (3.0, 3.0, "square/circle"), // width == height
    ];

    let center = Point::new(5.0, 10.0);

    for &(width, height, desc) in &test_cases {
        let shape = PadShape::Obround { width, height };

        for angle_deg in [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 135.0, 180.0, 270.0] {
            let bbox = shape.bounding_box(center, angle_deg);

            // Dense sample of obround boundary (stadium shape)
            let (spine_len, radius, is_horizontal) = if width >= height {
                (width - height, height / 2.0, true)
            } else {
                (height - width, width / 2.0, false)
            };

            let half_spine = spine_len / 2.0;
            let (sin_a, cos_a) = sin_cos_deg(angle_deg);

            // Sample points along spine + circular caps
            let n_samples = 3600;
            let mut sample_min_x = f64::INFINITY;
            let mut sample_max_x = f64::NEG_INFINITY;
            let mut sample_min_y = f64::INFINITY;
            let mut sample_max_y = f64::NEG_INFINITY;

            for s in 0..n_samples {
                let theta = (s as f64) * 2.0 * PI / (n_samples as f64);
                let (cap_center_x, cap_center_y) = if is_horizontal {
                    if theta.cos() >= 0.0 {
                        (half_spine, 0.0)
                    } else {
                        (-half_spine, 0.0)
                    }
                } else {
                    if theta.sin() >= 0.0 {
                        (0.0, half_spine)
                    } else {
                        (0.0, -half_spine)
                    }
                };

                let lx = cap_center_x + radius * theta.cos();
                let ly = cap_center_y + radius * theta.sin();

                // Rotate by angle_deg around center
                let rx = center.x + lx * cos_a - ly * sin_a;
                let ry = center.y + lx * sin_a + ly * cos_a;

                sample_min_x = sample_min_x.min(rx);
                sample_max_x = sample_max_x.max(rx);
                sample_min_y = sample_min_y.min(ry);
                sample_max_y = sample_max_y.max(ry);
            }

            // The analytical bbox must enclose ALL sampled points
            assert!(
                bbox.min.x <= sample_min_x + 1e-6,
                "Obround ({}) bbox min.x {} must <= sample min {}",
                desc,
                bbox.min.x,
                sample_min_x
            );
            assert!(
                bbox.max.x >= sample_max_x - 1e-6,
                "Obround ({}) bbox max.x {} must >= sample max {}",
                desc,
                bbox.max.x,
                sample_max_x
            );
            assert!(
                bbox.min.y <= sample_min_y + 1e-6,
                "Obround ({}) bbox min.y {} must <= sample min {}",
                desc,
                bbox.min.y,
                sample_min_y
            );
            assert!(
                bbox.max.y >= sample_max_y - 1e-6,
                "Obround ({}) bbox max.y {} must >= sample max {}",
                desc,
                bbox.max.y,
                sample_max_y
            );

            // And the difference between analytical extent and sample extent must be <= 1e-3 (sampling granularity)
            assert!(
                (bbox.max.x - sample_max_x) < 1e-3,
                "Tightness error in Obround max.x"
            );
            assert!(
                (bbox.max.y - sample_max_y) < 1e-3,
                "Tightness error in Obround max.y"
            );
        }
    }
}

// ============================================================================
// 3. Vector Arithmetic, Segment Projection & Zero-Handling
// ============================================================================

#[test]
fn probe_track_segment_point_distance_projections() {
    let seg = TrackSegment::new(
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        0.25,
        Layer::FCu,
        1,
    );

    // 1. Point projecting onto interior
    let dist_mid = seg.distance_to_point(Point::new(5.0, 3.0));
    assert!((dist_mid - 3.0).abs() < 1e-12);

    // 2. Point projecting before start
    let dist_before = seg.distance_to_point(Point::new(-4.0, 3.0));
    assert!((dist_before - 5.0).abs() < 1e-12); // sqrt((-4)^2 + 3^2) = 5

    // 3. Point projecting after end
    let dist_after = seg.distance_to_point(Point::new(14.0, 3.0));
    assert!((dist_after - 5.0).abs() < 1e-12);

    // 4. Zero-length track segment (start == end)
    let zero_seg = TrackSegment::new(
        Point::new(2.0, 3.0),
        Point::new(2.0, 3.0),
        0.25,
        Layer::FCu,
        1,
    );
    let dist_zero = zero_seg.distance_to_point(Point::new(5.0, 7.0));
    assert!((dist_zero - 5.0).abs() < 1e-12); // sqrt(3^2 + 4^2) = 5
}

#[test]
fn probe_point_zero_division_and_normalization() {
    // Normalizing zero vector
    let zero = Point::zero();
    let norm_zero = zero.normalize();
    assert_eq!(norm_zero, Point::zero());
    assert!(!norm_zero.x.is_nan() && !norm_zero.y.is_nan());

    // Normalizing very tiny vector (< 1e-12)
    let tiny = Point::new(1e-14, 1e-14);
    let norm_tiny = tiny.normalize();
    assert_eq!(norm_tiny, Point::zero());

    // Dividing point by 0.0
    let p = Point::new(10.0, 20.0);
    let div_zero = p / 0.0;
    assert_eq!(div_zero, Point::zero());

    // DivAssign by 0.0
    let mut p2 = Point::new(5.0, 15.0);
    p2 /= 0.0;
    assert_eq!(p2, Point::zero());
}

// ============================================================================
// 4. EdgeCuts Polygon: Non-Convex & Stress Scenarios
// ============================================================================

#[test]
fn probe_edge_cuts_non_convex_l_shape() {
    // L-shaped polygon:
    // (0,0) -> (10,0) -> (10,4) -> (4,4) -> (4,10) -> (0,10) -> (0,0)
    let vertices = [
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 4.0),
        Point::new(4.0, 4.0),
        Point::new(4.0, 10.0),
        Point::new(0.0, 10.0),
    ];
    let l_shape = EdgeCuts::from_vertices(&vertices);

    assert!(l_shape.is_closed(1e-6));

    // Area: 10 * 4 + 4 * 6 = 40 + 24 = 64
    assert!((l_shape.area() - 64.0).abs() < 1e-9);

    // Point inside bottom leg: (7, 2)
    assert!(l_shape.contains_point(Point::new(7.0, 2.0)));

    // Point inside vertical leg: (2, 7)
    assert!(l_shape.contains_point(Point::new(2.0, 7.0)));

    // Point inside junction: (2, 2)
    assert!(l_shape.contains_point(Point::new(2.0, 2.0)));

    // Point inside cutout of the L: (7, 7) -> MUST NOT BE CONTAINED!
    assert!(
        !l_shape.contains_point(Point::new(7.0, 7.0)),
        "Point in L-cutout (7,7) must be OUTSIDE"
    );

    // Point clearly outside: (12, 5)
    assert!(!l_shape.contains_point(Point::new(12.0, 5.0)));
}

// ============================================================================
// 5. Footprint Parametric Generators Deep Geometric Audit
// ============================================================================

#[test]
fn probe_footprint_tqfp32_pitch_and_pinout() {
    let mcu = tqfp32("U1", "ATmega328P");
    assert_eq!(mcu.pads.len(), 32);

    // Verify 0.80 mm pitch between all consecutive pins on the 4 sides
    // Left side: Pins 1..8
    for i in 1..8 {
        let p_curr = mcu.pad(&i.to_string()).unwrap();
        let p_next = mcu.pad(&(i + 1).to_string()).unwrap();
        let dy = (p_next.relative_pos.y - p_curr.relative_pos.y).abs();
        assert!(
            (dy - 0.80).abs() < 1e-6,
            "TQFP-32 Left side pitch between {} and {} must be 0.80, got {}",
            i,
            i + 1,
            dy
        );
    }

    // Bottom side: Pins 9..16
    for i in 9..16 {
        let p_curr = mcu.pad(&i.to_string()).unwrap();
        let p_next = mcu.pad(&(i + 1).to_string()).unwrap();
        let dx = (p_next.relative_pos.x - p_curr.relative_pos.x).abs();
        assert!(
            (dx - 0.80).abs() < 1e-6,
            "TQFP-32 Bottom side pitch between {} and {} must be 0.80, got {}",
            i,
            i + 1,
            dx
        );
    }
}

#[test]
fn probe_footprint_header_annular_rings_and_shapes() {
    let header = header_1x15("J1", "CONN");
    assert_eq!(header.pads.len(), 15);

    // Pin 1: Square pad for polarity indication
    assert!(matches!(header.pads[0].shape, PadShape::Rect { .. }));
    // Pins 2..15: Circular pads
    for p in &header.pads[1..] {
        assert!(matches!(p.shape, PadShape::Circle { .. }));
    }

    // Annular ring >= 0.35 mm (exceeding standard 0.15 mm DRC limit)
    for p in &header.pads {
        let ring = p.annular_ring().unwrap();
        assert!(
            ring >= 0.35,
            "Header pad annular ring must be >= 0.35 mm, got {}",
            ring
        );
    }
}

#[test]
fn probe_footprint_usb_c_receptacle_spec() {
    let usb = usb_c_16pin("J2", "USB-C");
    assert_eq!(usb.pads.len(), 16);

    // Check 12 SMD pads: pitch is 0.50 mm between adjacent pins
    let mut smd_x_coords: Vec<f64> = usb
        .pads
        .iter()
        .filter(|p| p.pad_type.is_smd())
        .map(|p| p.relative_pos.x)
        .collect();
    smd_x_coords.sort_by(|a, b| a.partial_cmp(b).unwrap());

    assert_eq!(smd_x_coords.len(), 12);
    for i in 0..11 {
        let diff = (smd_x_coords[i + 1] - smd_x_coords[i]).abs();
        assert!(
            (diff - 0.50).abs() < 1e-6,
            "USB-C SMD pad pitch between adjacent contacts must be 0.50 mm, got {}",
            diff
        );
    }

    // Check 4 THT shell tabs
    let tht_pads: Vec<&Pad> = usb.pads.iter().filter(|p| p.pad_type.is_through_hole()).collect();
    assert_eq!(tht_pads.len(), 4);
    for tht in tht_pads {
        assert_eq!(tht.pad_type.drill_diameter(), Some(1.00));
        assert!(tht.annular_ring().unwrap() >= 0.35);
    }
}

// ============================================================================
// 6. Netlist Ingestion Conflict Semantics & Board Validation Errors
// ============================================================================

#[test]
fn probe_netlist_ingestion_idempotence_and_conflict() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.add_footprint(resistor_0805("R1", "10k"));

    // 1. Assigning net GND to pin 1 succeeds
    assert!(board.assign_pad_net("R1", "1", "GND").is_ok());

    // 2. Assigning same net GND to pin 1 again must SUCCEED idempotently
    assert!(board.assign_pad_net("R1", "1", "GND").is_ok());

    // 3. Assigning different net VCC to pin 1 must FAIL with DuplicatePinAssignment
    let err = board.assign_pad_net("R1", "1", "VCC").unwrap_err();
    match err {
        IngestionError::DuplicatePinAssignment {
            component_ref,
            pin_id,
            existing_net_name,
            new_net_name,
        } => {
            assert_eq!(component_ref, "R1");
            assert_eq!(pin_id, "1");
            assert_eq!(existing_net_name, "GND");
            assert_eq!(new_net_name, "VCC");
        }
        _ => panic!("Expected DuplicatePinAssignment"),
    }
}

#[test]
fn probe_board_validation_catches_all_invalidities() {
    let mut board = PcbBoard::new(100.0, 100.0);

    // 1. Footprint with pad out of bounds
    let oob_fp = resistor_0805("R_OOB", "10k").at_position(Point::new(105.0, 50.0));
    board.add_footprint(oob_fp);

    // 2. Footprint with duplicate pin number
    let dup_fp = Footprint::new(
        "U_DUP",
        "CHIP",
        "CUSTOM",
        Point::new(50.0, 50.0),
        0.0,
        vec![
            Pad::new_smd("1", "A", Point::new(-1.0, 0.0), PadShape::Circle { diameter: 1.0 }, Layer::FCu),
            Pad::new_smd("1", "B", Point::new(1.0, 0.0), PadShape::Circle { diameter: 1.0 }, Layer::FCu),
        ],
        vec![],
    );
    board.add_footprint(dup_fp);

    // 3. Footprint with empty pads
    let empty_fp = Footprint::new(
        "U_EMPTY",
        "NONE",
        "EMPTY",
        Point::new(30.0, 30.0),
        0.0,
        vec![],
        vec![],
    );
    board.add_footprint(empty_fp);

    // 4. Footprint with invalid THT drill (drill >= outer)
    let invalid_tht_fp = Footprint::new(
        "J_BAD",
        "CONN",
        "BAD_THT",
        Point::new(20.0, 20.0),
        0.0,
        vec![Pad::new_tht("1", "P", Point::zero(), PadShape::Circle { diameter: 1.0 }, 1.5, 1.0)],
        vec![],
    );
    board.add_footprint(invalid_tht_fp);

    let report = board.validate();
    assert!(!report.is_valid());

    let has_oob = report.errors.iter().any(|e| matches!(e, ValidationError::PadOutOfBounds { .. }));
    let has_dup = report.errors.iter().any(|e| matches!(e, ValidationError::DuplicatePinNumber { .. }));
    let has_empty = report.errors.iter().any(|e| matches!(e, ValidationError::EmptyFootprint { .. }));
    let has_tht = report.errors.iter().any(|e| matches!(e, ValidationError::InvalidThroughHoleDrill { .. }));

    assert!(has_oob, "Must detect PadOutOfBounds");
    assert!(has_dup, "Must detect DuplicatePinNumber");
    assert!(has_empty, "Must detect EmptyFootprint");
    assert!(has_tht, "Must detect InvalidThroughHoleDrill");
}

// ============================================================================
// 7. Advanced Net Mapping, Pin Identification, and Ratsnest EMST
// ============================================================================

#[test]
fn probe_pin_identification_by_number_or_name() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.add_footprint(tqfp32("U1", "ATmega328P"));

    // Pin 1 is named "PD3"
    // Pin 4 is named "VCC"
    // Pin 3 is named "GND"
    assert!(board.assign_pad_net("U1", "PD3", "DATA").is_ok());
    assert!(board.assign_pad_net("U1", "4", "VCC").is_ok());
    assert!(board.assign_pad_net("U1", "GND", "GND").is_ok());

    let u1 = board.find_footprint("U1").unwrap();
    let data_id = board.net_names.iter().find(|(_, name)| *name == "DATA").unwrap().0;
    let vcc_id = board.net_names.iter().find(|(_, name)| *name == "VCC").unwrap().0;

    assert_eq!(u1.pad("1").unwrap().net, Some(*data_id));
    assert_eq!(u1.pad("PD3").unwrap().net, Some(*data_id));
    assert_eq!(u1.pad("4").unwrap().net, Some(*vcc_id));
    assert_eq!(u1.pad("VCC").unwrap().net, Some(*vcc_id));
}

#[test]
fn probe_schematic_netlist_apply_end_to_end() {
    let mut board = PcbBoard::new(60.0, 40.0);
    board.add_footprint(resistor_0805("R1", "10k").at_position(Point::new(10.0, 20.0)));
    board.add_footprint(resistor_0805("R2", "10k").at_position(Point::new(30.0, 20.0)));
    board.add_footprint(led_0805("D1", "RED").at_position(Point::new(50.0, 20.0)));

    let mut netlist = SchematicNetlist::new();
    netlist.add_net("NET_VCC", &[("R1", "1")]);
    netlist.add_net("NET_MID", &[("R1", "2"), ("R2", "1")]);
    netlist.add_net("NET_LED", &[("R2", "2"), ("D1", "A")]); // D1 pin 2 is "A"
    netlist.add_net("NET_GND", &[("D1", "K")]);             // D1 pin 1 is "K"

    assert!(board.apply_netlist(&netlist).is_ok());

    let mid_id = board.get_or_create_net_id("NET_MID");
    assert_eq!(board.find_footprint("R1").unwrap().pad("2").unwrap().net, Some(mid_id));
    assert_eq!(board.find_footprint("R2").unwrap().pad("1").unwrap().net, Some(mid_id));

    let led_id = board.get_or_create_net_id("NET_LED");
    assert_eq!(board.find_footprint("R2").unwrap().pad("2").unwrap().net, Some(led_id));
    assert_eq!(board.find_footprint("D1").unwrap().pad("A").unwrap().net, Some(led_id));

    // Verify ratsnest produces 1 line for 2-pin nets NET_MID and NET_LED
    let lines = compute_ratsnest(&board);
    let mid_lines = lines.iter().filter(|l| l.net == mid_id).count();
    let led_lines = lines.iter().filter(|l| l.net == led_id).count();
    assert_eq!(mid_lines, 1);
    assert_eq!(led_lines, 1);
}

#[test]
fn probe_ratsnest_emst_copper_clustering_isolation() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let fp1 = resistor_0805("R1", "1k").at_position(Point::new(10.0, 25.0));
    let fp2 = resistor_0805("R2", "1k").at_position(Point::new(30.0, 25.0));
    board.add_footprint(fp1);
    board.add_footprint(fp2);

    let net_id = board.get_or_create_net_id("SIG");
    board.assign_pad_net("R1", "2", "SIG").unwrap();
    board.assign_pad_net("R2", "1", "SIG").unwrap();

    let p1 = board.find_footprint("R1").unwrap().pad_world_pos("2").unwrap();
    let p2 = board.find_footprint("R2").unwrap().pad_world_pos("1").unwrap();

    // Before routing: exactly 1 ratsnest line
    let lines_unrouted = compute_ratsnest(&board);
    assert_eq!(lines_unrouted.len(), 1);

    // Route copper trace belonging to DIFFERENT net -> ratsnest line MUST REMAIN
    let track_wrong = TrackSegment::new(p1, p2, 0.25, Layer::FCu, 9999);
    board.add_track(track_wrong);
    let lines_wrong = compute_ratsnest(&board);
    assert_eq!(lines_wrong.len(), 1, "Track of different net must not eliminate ratsnest");

    // Route copper trace belonging to MATCHING net -> ratsnest line MUST BE ELIMINATED
    let track_correct = TrackSegment::new(p1, p2, 0.25, Layer::FCu, net_id);
    board.add_track(track_correct);
    let lines_routed = compute_ratsnest(&board);
    assert_eq!(lines_routed.len(), 0, "Track of matching net must eliminate ratsnest line");
}

