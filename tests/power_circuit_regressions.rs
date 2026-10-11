use netlist_shatter_viz::pcb::*;

#[test]
fn filtered_adc_supply_survives_core_completion_and_rejects_bypass_faults() {
    let mut board = create_framework_card_board();
    board.footprints.retain(|f| {
        !f.reference
            .strip_prefix('C')
            .and_then(|n| n.parse::<u32>().ok())
            .is_some_and(|n| (20..=31).contains(&n))
    });
    let adc = board.get_or_create_net("ADC_VREF");
    let vdd = board.get_or_create_net("3V3");
    let gnd = board.get_or_create_net("GND");
    board
        .find_footprint_mut("U1")
        .unwrap()
        .pads
        .iter_mut()
        .find(|p| p.pin_number == "43")
        .unwrap()
        .net = Some(adc);
    rp2040::complete_core(&mut board);
    assert_eq!(
        board.find_footprint("U1").unwrap().pad("43").unwrap().net,
        Some(adc)
    );
    assert_eq!(
        board.find_footprint("C27").unwrap().pad("1").unwrap().net,
        Some(adc)
    );
    assert!(
        check_electrical(&board)
            .iter()
            .any(|i| i.message.contains("supply filter"))
    );

    board.add_footprint(resistor_0805("R_ADC", "200R"));
    board.add_footprint(capacitor_0805("C_ADC", "2.2uF"));
    board.connect("R_ADC", "1", vdd);
    board.connect("R_ADC", "2", adc);
    board.connect("C_ADC", "1", adc);
    board.connect("C_ADC", "2", gnd);
    assert!(
        check_electrical(&board).is_empty(),
        "{:?}",
        check_electrical(&board)
    );

    let mut bypassed = board.clone();
    bypassed
        .find_footprint_mut("U1")
        .unwrap()
        .pads
        .iter_mut()
        .find(|p| p.pin_number == "43")
        .unwrap()
        .net = Some(vdd);
    assert!(
        check_electrical(&bypassed)
            .iter()
            .any(|i| i.message.contains("pin 43"))
    );

    board.footprints.retain(|f| f.reference != "C_ADC");
    assert!(
        check_electrical(&board)
            .iter()
            .any(|i| i.message.contains("2.2uF"))
    );
}

#[test]
fn pico_power_and_sensing_faults_are_rejected_before_routing() {
    let board = create_pico_ultimate_board();
    assert!(
        check_electrical(&board).is_empty(),
        "{:?}",
        check_electrical(&board)
    );
    for (reference, expected) in [
        ("R11", "PROG"),
        ("L1", "2.2uH"),
        ("R22", "GPIO24"),
        ("C8", "ADC_VREF"),
        ("C3", "grounded 47uF"),
    ] {
        let mut faulty = board.clone();
        faulty.footprints.retain(|f| f.reference != reference);
        assert!(
            check_electrical(&faulty)
                .iter()
                .any(|i| i.message.contains(expected)),
            "missing {reference}"
        );
    }
    for (reference, expected) in [("Q1", "load switch"), ("Q2", "GPIO29")] {
        let mut faulty = board.clone();
        let f = faulty.find_footprint_mut(reference).unwrap();
        let source = f.pads.iter().position(|p| p.pin_number == "2").unwrap();
        let drain = f.pads.iter().position(|p| p.pin_number == "3").unwrap();
        let net = f.pads[source].net;
        f.pads[source].net = f.pads[drain].net;
        f.pads[drain].net = net;
        assert!(
            check_electrical(&faulty)
                .iter()
                .any(|i| i.message.contains(expected)),
            "reversed {reference}"
        );
    }
    let mut faulty = board;
    let vdd = faulty.get_or_create_net("3V3");
    faulty
        .find_footprint_mut("R1")
        .unwrap()
        .pads
        .iter_mut()
        .find(|p| p.pin_number == "2")
        .unwrap()
        .net = Some(vdd);
    assert!(
        check_electrical(&faulty)
            .iter()
            .any(|i| i.message.contains("STAT LED"))
    );
}
