use netlist_shatter_viz::pcb::*;
fn example() -> PcbBoard {
    let mut b = PcbBoard::new(45., 30.);
    b.register_net(1, "SIG");
    b.register_net(2, "GND");
    for (r, x, y) in [("J1", 5., 5.), ("R1", 32., 22.), ("C1", 32., 26.)] {
        let mut f = resistor_0805(r, "1k").at_position(Point::new(x, y));
        f.pads[0].net = Some(1);
        f.pads[1].net = Some(2);
        b.add_footprint(f);
    }
    b
}
#[test]
fn placement_preserves_circuit_pins_and_rigid_groups() {
    let mut b = example();
    let original = b.clone();
    let c = PlacementConstraints {
        max_move_mm: None,
        fixed: vec!["J1".into()],
        groups: vec![vec!["R1".into(), "C1".into()]],
    };
    let r = optimize_placement(&mut b, &c, 0).unwrap();
    assert!(r.after_hpwl_mm < r.before_hpwl_mm);
    assert!(r.moved_components > 0);
    assert!(r.placement_issues.is_empty());
    assert_eq!(b.footprints[0], original.footprints[0]);
    assert_eq!(
        b.footprints[2].position - b.footprints[1].position,
        Point::new(0., 4.)
    );
    for (f, o) in b.footprints.iter().zip(&original.footprints) {
        assert_eq!(f.pads, o.pads);
        assert_eq!(f.rotation, o.rotation);
    }
    assert_eq!(b.net_names, original.net_names);
    let mut repeat = original;
    optimize_placement(&mut repeat, &c, 0).unwrap();
    assert_eq!(b, repeat);
}
#[test]
fn rejects_invalid_constraints_without_mutation() {
    let mut b = example();
    let original = b.clone();
    for c in [
        PlacementConstraints {
            max_move_mm: None,
            fixed: vec!["missing".into()],
            groups: vec![],
        },
        PlacementConstraints {
            max_move_mm: None,
            fixed: vec![],
            groups: vec![vec!["R1".into()], vec!["R1".into()]],
        },
    ] {
        assert!(optimize_placement(&mut b, &c, 0).is_err());
        assert_eq!(b, original);
    }
}
#[test]
fn changed_placement_removes_stale_routes_but_noop_retains_them() {
    let mut b = example();
    b.add_track(TrackSegment::new(
        Point::new(5., 5.),
        Point::new(32., 22.),
        0.2,
        Layer::FCu,
        1,
    ));
    let c = PlacementConstraints {
        max_move_mm: None,
        fixed: vec!["J1".into(), "R1".into(), "C1".into()],
        groups: vec![],
    };
    assert_eq!(optimize_placement(&mut b, &c, 0).unwrap().removed_tracks, 0);
    assert_eq!(b.tracks.len(), 1);
    let c = PlacementConstraints {
        max_move_mm: None,
        fixed: vec!["J1".into()],
        groups: vec![],
    };
    assert_eq!(optimize_placement(&mut b, &c, 0).unwrap().removed_tracks, 1);
    assert!(b.tracks.is_empty());
}
#[test]
fn existing_conflicts_are_reported_and_fixed_group_is_immovable() {
    let mut b = example();
    b.footprints[1].position = b.footprints[0].position;
    let c = PlacementConstraints {
        max_move_mm: None,
        fixed: vec!["J1".into()],
        groups: vec![vec!["J1".into(), "R1".into(), "C1".into()]],
    };
    let original = b.clone();
    let r = optimize_placement(&mut b, &c, 0).unwrap();
    assert!(!r.placement_issues.is_empty());
    assert_eq!(b, original);
}

#[test]
fn separating_overlapping_parts_takes_priority_over_shorter_wires() {
    let mut b = PcbBoard::new(20., 20.);
    b.register_net(1, "SIG");
    b.register_net(2, "GND");
    for reference in ["J1", "C1"] {
        let mut f = resistor_0805(reference, "1k").at_position(Point::new(10., 10.));
        f.pads[0].net = Some(1);
        f.pads[1].net = Some(2);
        b.add_footprint(f);
    }
    let original = b.clone();
    let constraints = PlacementConstraints { fixed: vec!["J1".into()], groups: vec![], max_move_mm: Some(4.) };
    let result = optimize_placement(&mut b, &constraints, 0).unwrap();
    assert!(result.placement_issues.is_empty());
    assert!(result.after_hpwl_mm > result.before_hpwl_mm);
    assert_eq!(b.footprints[0], original.footprints[0]);
    assert_eq!(b.footprints[1].pads, original.footprints[1].pads);
    assert!(b.footprints[1].position.distance_to(original.footprints[1].position) <= 4.);
}

#[test]
fn edge_setback_applies_to_pad_copper_while_bodies_still_block_neighbors() {
    let mut b = PcbBoard::new(26., 30.);
    b.add_footprint(header_1x10("HDR1", "EXPANSION")
        .at_position(Point::new(1.57, 27.5)).with_rotation(270.));
    // The real 25.4mm body fits the board but cannot fit a fictitious
    // 0.4mm body setback on both ends. The lands have 0.67mm clearance.
    assert!(placement::placement_issues(&b).is_empty());
    b.footprints[0].position.x -= 0.5;
    assert!(placement::placement_issues(&b).iter().any(|s| s.contains("pad copper")));
    b.footprints[0].position.x += 0.5;
    b.add_footprint(resistor_0805("R1", "1k").at_position(Point::new(13., 27.5)));
    assert!(placement::placement_issues(&b).iter().any(|s| s.contains("envelopes overlap")));
}

#[test]
fn automatic_route_cleanup_repairs_external_fanout_without_moving_components() {
    let mut b = create_rp2040_bringup_board();
    import_specctra_ses(
        &mut b,
        include_str!("fixtures/rp2040-fanout-before-cleanup.ses"),
    )
    .unwrap();
    let fps = b.footprints.clone();
    let nets = b.net_names.clone();
    assert!(b.tracks.iter().any(|t| t.width < 0.15));
    let r = cleanup_route(&mut b, &prototype_rules());
    assert!(r.clean, "{r:?}");
    assert_eq!(r.widths_normalized, 33);
    assert!(r.vertices_adjusted > 0);
    assert_eq!(b.footprints, fps);
    assert_eq!(b.net_names, nets);
    assert!(validate_board(&b, &prototype_rules()).software_checks_pass);
}
#[test]
fn route_cleanup_does_not_hide_missing_connections() {
    let mut b = create_rp2040_bringup_board();
    let original = b.clone();
    assert!(!cleanup_route(&mut b, &prototype_rules()).clean);
    assert_eq!(b, original);
}
#[test]
fn movement_budget_limits_translations_and_zero_is_a_true_baseline() {
    let mut b = example();
    let original = b.clone();
    let c = PlacementConstraints {
        max_move_mm: Some(0.0),
        ..Default::default()
    };
    optimize_placement(&mut b, &c, 1).unwrap();
    assert_eq!(b, original);
    let c = PlacementConstraints {
        max_move_mm: Some(0.75),
        ..Default::default()
    };
    optimize_placement(&mut b, &c, 1).unwrap();
    for (f, o) in b.footprints.iter().zip(original.footprints) {
        assert!(f.position.distance_to(o.position) <= 0.75000001);
    }
}

#[test]
fn bottom_parts_are_identified_in_engineering_placement_output() {
    let mut b = example();
    for p in &mut b.footprints[1].pads {
        p.pad_type = PadType::Smd { layer: Layer::BCu };
    }
    let cpl = generate_cpl_from_board(&b);
    assert_eq!(
        cpl.iter().find(|e| e.designator == "R1").unwrap().layer,
        "Bottom"
    );
    assert_eq!(
        cpl.iter().find(|e| e.designator == "J1").unwrap().layer,
        "Top"
    );
}
#[test]
fn ses_round_dot_is_preserved_then_removed_only_when_covered() {
    let mut b = PcbBoard::new(10., 10.);
    b.register_net(1, "N");
    let ses = "(session x (routes (resolution mm 1) (network_out (net N (wire (path F.Cu 0.15 2 2)) (wire (path F.Cu 0.1 2 2 5 2))))))";
    import_specctra_ses(&mut b, ses).unwrap();
    assert_eq!(b.tracks.len(), 2);
    assert!(cleanup_route(&mut b, &prototype_rules()).clean);
    assert_eq!(b.tracks.len(), 1);
    assert_eq!(b.tracks[0].width, 0.15);
}
