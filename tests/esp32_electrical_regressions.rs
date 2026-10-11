use netlist_shatter_viz::pcb::*;

#[test]
fn camera_power_bus_and_autoreset_faults_fail_the_shared_electrical_gate() {
    let board = create_esp32_cam_pro_board();
    assert!(check_electrical(&board).is_empty());
    for (reference, pin, wrong_net) in [
        ("J2", "4", "CAM_AVDD"),
        ("J2", "15", "3V3_SYS"),
        ("U3", "3", "GND"),
        ("U4", "6", "3V3_SYS"),
        ("U4", "26", "UART_TX0"),
        ("U2", "3", "USB_VBUS"),
        ("Q1", "2", "GND"),
        ("J3", "5", "SD_MISO"),
    ] {
        let mut broken = board.clone();
        let net = broken.get_or_create_net(wrong_net);
        broken
            .find_footprint_mut(reference)
            .unwrap()
            .pad_mut(pin)
            .unwrap()
            .net = Some(net);
        assert!(
            !check_electrical(&broken).is_empty(),
            "{reference}.{pin} fault escaped the gate"
        );
    }
    for reference in [
        "U4", "U5", "R3", "R7", "R10", "R15", "R16", "C7", "C13", "C15",
    ] {
        let mut broken = board.clone();
        broken.footprints.retain(|f| f.reference != reference);
        assert!(
            !check_electrical(&broken).is_empty(),
            "Missing {reference} escaped the gate"
        );
    }
}

#[test]
fn complete_camera_draft_retains_routing_and_power_qualification_holds() {
    let board = create_esp32_cam_pro_board();
    let report = validate_board(&board, &prototype_rules());
    assert!(report.electrical_issues.is_empty());
    assert!(!report.software_checks_pass);
    assert!(!report.hardware_validated);
    assert!(!report.drc.unrouted_nets.is_empty());
    for requirement in ["External", "thermal", "antenna", "USB"] {
        assert!(
            report.limitations.iter().any(|s| s.contains(requirement)),
            "Missing {requirement} hold"
        );
    }
}
