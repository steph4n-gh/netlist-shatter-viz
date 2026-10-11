use netlist_shatter_viz::pcb::*;

#[test]
fn reviewed_rp2040_session_passes_the_software_gate() {
    let mut b = create_rp2040_bringup_board();
    import_specctra_ses(&mut b, include_str!("../reference/routes/bringup.ses")).unwrap();
    let report = validate_board(&b, &prototype_rules());
    assert!(report.software_checks_pass, "{report:?}");
    assert!(!report.hardware_validated);
    assert_eq!(b.tracks.len(), 519);
    assert_eq!(b.vias.len(), 43);
    assert!(b.tracks.iter().all(|t| t.width >= 0.15 - 1e-6));
}

#[test]
fn rp2040_physical_pins_and_all_supplies_are_correct() {
    let b = create_rp2040_bringup_board();
    let f = b.find_footprint("U1").unwrap();
    for (pin, name, net) in [
        ("45", "VREG_VOUT", "1V1_CORE"),
        ("46", "USB_DM", "USB_DM"),
        ("47", "USB_DP", "USB_DP"),
        ("51", "QSPI_SD3", "QSPI_D3"),
        ("54", "QSPI_SD2", "QSPI_D2"),
        ("56", "QSPI_SS", "QSPI_CS"),
    ] {
        let p = f.pads.iter().find(|p| p.pin_number == pin).unwrap();
        assert_eq!(p.pin_name, name);
        assert_eq!(b.get_net_name(p.net.unwrap()), Some(net));
    }
    assert!(check_electrical(&b).is_empty());
}

#[test]
fn missing_supply_is_an_electrical_failure_even_before_routing() {
    let mut b = create_rp2040_bringup_board();
    b.find_footprint_mut("U1")
        .unwrap()
        .pads
        .iter_mut()
        .find(|p| p.pin_number == "48")
        .unwrap()
        .net = None;
    assert!(
        check_electrical(&b)
            .iter()
            .any(|i| i.message.contains("pin 48"))
    );
}

#[test]
fn clock_boot_reset_and_led_faults_fail_electrical_review_without_copper() {
    for (reference, reason) in [
        ("R5", "Crystal drive"),
        ("R6", "BOOTSEL"),
        ("R7", "RUN"),
        ("R8", "Status LED"),
        ("C4", "load capacitor"),
    ] {
        let mut b = create_rp2040_bringup_board();
        b.footprints.retain(|f| f.reference != reference);
        assert!(
            check_electrical(&b)
                .iter()
                .any(|i| i.message.contains(reason)),
            "missing {reference}"
        );
    }
    let mut b = create_framework_card_board();
    let cs = b
        .find_footprint("U1")
        .unwrap()
        .pads
        .iter()
        .find(|p| p.pin_number == "56")
        .unwrap()
        .net;
    b.find_footprint_mut("SW1")
        .unwrap()
        .pads
        .iter_mut()
        .find(|p| p.pin_number == "2")
        .unwrap()
        .net = cs;
    assert!(
        check_electrical(&b)
            .iter()
            .any(|i| i.component == "SW1" && i.message.contains("isolation"))
    );
    let mut b = create_framework_card_board();
    // Reverse the actual assignments without changing the physical pin identities.
    let f = b.find_footprint_mut("LED1").unwrap();
    let net = f.pads[0].net;
    f.pads[0].net = f.pads[1].net;
    f.pads[1].net = net;
    assert!(check_electrical(&b).iter().any(|i| i.component == "LED1"));
}

#[test]
#[should_panic(expected = "Missing pin")]
fn misspelled_pin_cannot_silently_disappear() {
    let mut b = create_rp2040_bringup_board();
    b.connect("U1", "999", 1);
}

#[test]
fn copper_clearance_applies_inside_a_footprint() {
    let mut b = PcbBoard::new(20.0, 20.0);
    b.register_net(1, "A");
    b.register_net(2, "B");
    let mut f = resistor_0805("R1", "1k").at_position(Point::new(10.0, 10.0));
    f.pads[0].net = Some(1);
    f.pads[1].net = Some(2);
    f.pads[1].relative_pos = f.pads[0].relative_pos;
    b.add_footprint(f);
    assert!(!run_drc(&b, &prototype_rules()).short_circuits.is_empty());
}

#[test]
fn failed_routing_preserves_previous_copper() {
    let mut b = create_rp2040_bringup_board();
    b.add_track(TrackSegment::new(
        Point::new(1.0, 1.0),
        Point::new(2.0, 2.0),
        0.2,
        Layer::FCu,
        1,
    ));
    let before = serde_json::to_string(&b.tracks).unwrap();
    for resolution in [f64::NAN, 0.0, 1e-12] {
        assert!(
            route_board(
                &mut b,
                &RouterConfig {
                    grid_resolution: resolution,
                    ..prototype_rules()
                }
            )
            .is_err()
        );
        assert_eq!(serde_json::to_string(&b.tracks).unwrap(), before);
    }
    // Valid settings, but an outside pad makes a small board unroutable.
    let mut blocked = PcbBoard::new(4.0, 4.0);
    blocked.register_net(1, "A");
    let mut f = resistor_0805("R", "1k").at_position(Point::new(-3.0, 2.0));
    f.pads[0].net = Some(1);
    f.pads[1].net = Some(1);
    blocked.add_footprint(f);
    blocked.tracks = b.tracks.clone();
    assert!(route_board(&mut blocked, &prototype_rules()).is_err());
    assert_eq!(serde_json::to_string(&blocked.tracks).unwrap(), before);
}

#[test]
fn session_import_rejects_stale_placement_without_mutation() {
    let mut b = create_rp2040_bringup_board();
    b.find_footprint_mut("U1").unwrap().position.x += 1.0;
    let before = b.tracks.clone();
    assert!(import_specctra_ses(&mut b, include_str!("../reference/routes/bringup.ses")).is_err());
    assert_eq!(b.tracks, before);
}

#[test]
fn dsn_mm_coordinates_are_not_scaled_by_resolution() {
    let b = PcbBoard::new(60.0, 42.0);
    let dsn = export_specctra_dsn(&b, &SpecctraDsnExportConfig::default());
    assert!(dsn.contains("(rect pcb 0 0 60 42)"));
    let parsed = parse_specctra_dsn(&dsn).unwrap();
    assert_eq!(parsed.bounds.width(), 60.0);
}

#[test]
fn dsn_does_not_merge_same_named_packages_with_different_layers() {
    let mut b = PcbBoard::new(20.0, 20.0);
    let a = resistor_0805("R1", "a").at_position(Point::new(5.0, 5.0));
    let mut c = a.clone();
    c.reference = "R2".into();
    c.position = Point::new(15.0, 15.0);
    for p in &mut c.pads {
        p.pad_type = PadType::Smd { layer: Layer::BCu };
    }
    b.add_footprint(a);
    b.add_footprint(c);
    let text = dsn_for_board(&b, "test", &prototype_rules());
    let (round, _) = parse_specctra_dsn(&text).unwrap().to_board().unwrap();
    for (name, layer) in [("R1", Layer::FCu), ("R2", Layer::BCu)] {
        assert!(
            round
                .find_footprint(name)
                .unwrap()
                .pads
                .iter()
                .all(|p| p.pad_type == PadType::Smd { layer })
        );
    }
}

#[test]
fn rotated_pad_export_preserves_orientation_and_thickness() {
    let mut b = PcbBoard::new(30.0, 20.0);
    b.thickness = 0.8;
    b.add_footprint(
        resistor_0805("R1", "1k")
            .at_position(Point::new(10.0, 10.0))
            .with_rotation(45.0),
    );
    assert!(export_gerber_layer(&b, Layer::FCu).contains("21,1,1.000000,1.300000,0,0,-45.000000*"));
    let pcb = export_kicad_pcb(&b);
    assert!(pcb.contains("(thickness 0.8)"));
    assert!(pcb.contains("(at 10 10 -45)"));
}

#[test]
fn legacy_designs_are_not_mislabeled_as_validated_hardware() {
    for name in ["framework", "pico", "esp32", "m64"] {
        let b = board_by_name(name).unwrap();
        let r = validate_board(&b, &prototype_rules());
        assert!(!r.software_checks_pass, "{name}");
        if name == "framework" || name == "pico" || name == "esp32" {
            assert!(r.electrical_issues.is_empty(), "{:?}", r.electrical_issues);
            assert!(!r.drc.unrouted_nets.is_empty());
        } else {
            assert!(!r.electrical_issues.is_empty(), "{name}");
        }
        assert!(!r.hardware_validated);
    }
}

#[test]
fn dsn_pad_names_survive_external_decimal_suffix_normalization() {
    let b = create_rp2040_bringup_board();
    let dsn = dsn_for_board(&b, "bringup", &prototype_rules());
    let parsed = parse_specctra_dsn(&dsn).unwrap();
    let ids: Vec<_> = parsed.padstacks.keys().collect();
    assert!(
        ids.iter().all(|id| !id.contains('.')),
        "Freerouting strips decimal suffixes from names"
    );
    assert!(dsn.contains("rect_0p700000x0p200000_smd_FCu"));
    assert!(dsn.contains("rect_0p200000x0p700000_smd_FCu"));
    assert!(dsn.contains("rect_0p600000x0p600000_smd_FCu"));
}
