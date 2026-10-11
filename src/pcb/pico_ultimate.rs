//! RP2040 development board with external protected-1S battery power.
//! Circuit sources: Raspberry Pi Pico Rev3 schematic, Microchip DS20001984H,
//! Richtek DS6150A/B-06, Diodes DMG2305UX/DMG1012T, Worldsemi WS2812B-V6.
//! 72x30mm is a development-board outline, not a Pico drop-in outline.
//! External pack protection/temperature, USB current budget, power-loop/land-pattern
//! qualification and RGB operation at the regulator's worst-case voltage remain holds.
use crate::pcb::assembly::{BomEntry, CplEntry, generate_bom_from_board, generate_cpl_from_board};
use crate::pcb::core::{PcbBoard, Point};
use crate::pcb::footprint::*;
use crate::pcb::router::{RouterConfig, RoutingError, route_board};

fn connect(board: &mut PcbBoard, name: &str, pins: &[(&str, &str)]) {
    for (reference, pin) in pins {
        board
            .assign_pad_net(reference, pin, name)
            .expect("Invalid Pico netlist");
    }
}

/// Complete engineering netlist. Both header pin1s face USB; HDR1 maps Pico1..20,
/// HDR2 maps Pico40..21. Row spacing17.78mm, pitch2.54mm; outline remains72x30mm.
pub fn create_pico_ultimate_board() -> PcbBoard {
    let mut board = PcbBoard::new(72.0, 30.0);
    // Place large/fixed parts first. The shared passive helper conservatively avoids
    // these physical envelopes, including both header bodies.
    for f in [
        usb_c_16pin("J1", "USB-C")
            .at_position(Point::new(4.5, 15.0))
            .with_rotation(270.0),
        header_1x20("HDR1", "PICO_PINS_1_20")
            .at_position(Point::new(12.0, 6.11))
            .with_rotation(270.0),
        header_1x20("HDR2", "PICO_PINS_40_21")
            .at_position(Point::new(12.0, 23.89))
            .with_rotation(270.0),
        jst_ph_2pin("J2", "PROTECTED_1S_PACK").at_position(Point::new(16.0, 15.0)),
        sot23_5("U3", "MCP73831T-2ACI/OT").at_position(Point::new(22.0, 10.0)),
        diode_sod123("D1", "MBR120VLSFT1G").at_position(Point::new(23.0, 16.0)),
        dmg2305ux_sot23("Q1", "DMG2305UX-7").at_position(Point::new(23.0, 20.0)),
        rt6150b_wdfn10("U4", "RT6150B-33GQW").at_position(Point::new(28.0, 13.0)),
        vls252012hbx("L1", "2.2uH").at_position(Point::new(28.0, 17.0)),
        qfn56("U1", "RP2040-B2").at_position(Point::new(41.0, 15.0)),
        soic8("U2", "W25Q128JVS").at_position(Point::new(52.0, 15.0)),
        crystal_smd_3225("Y1", "ABM8-272-T3 12MHz").at_position(Point::new(37.0, 20.5)),
        ws2812b_v6_5050("LED1", "WS2812B-V6").at_position(Point::new(62.0, 15.0)),
        jst_sh_4pin("J3", "QWIIC-I2C")
            .at_position(Point::new(68.0, 15.0))
            .with_rotation(90.0),
        tactile_switch_3x4("SW1", "RESET").at_position(Point::new(65.0, 3.0)),
        tactile_switch_3x4("SW2", "BOOTSEL").at_position(Point::new(65.0, 27.0)),
        header_1x3("J4", "SWD").at_position(Point::new(69.0, 21.0)),
        dmg1012t_sot523("Q2", "DMG1012T-7").at_position(Point::new(34.0, 10.0)),
        led_0805("LED2", "USER_LED").at_position(Point::new(64.0, 10.0)),
        led_0805("LED3", "CHG_LED").at_position(Point::new(64.0, 20.0)),
    ] {
        board.add_footprint(f);
    }
    // Larger supply MLCCs are not replaced by miniature bypass footprints.
    for (r, value, x, y) in [
        ("C1", "10uF", 18.0, 10.0),
        // Leave room for 0.15mm traces and clearance to escape the converter leads.
        ("C2", "10uF", 24.5, 13.0),
        ("C3", "47uF", 31.5, 13.0),
        ("C4", "1uF", 35.0, 10.0),
        ("C7", "10uF", 18.0, 20.0),
        ("C8", "2.2uF", 33.0, 20.0),
    ] {
        crate::pcb::rp2040::add_passive(
            &mut board,
            capacitor_0805(r, value).at_position(Point::new(x, y)),
        );
    }
    for (r, value, x, y) in [
        ("R1", "2.0k", 61.0, 20.0),
        ("R3", "10k", 42.0, 9.0),
        ("R5", "5.1k", 10.0, 10.0),
        ("R6", "5.1k", 10.0, 20.0),
        ("R9", "27R", 12.0, 12.0),
        ("R10", "27R", 12.0, 18.0),
        ("R11", "10k", 22.0, 8.0),
        ("R12", "100k", 23.0, 22.0),
        ("R13", "1k", 38.0, 20.0),
        ("R14", "1k", 57.0, 20.0),
        ("R15", "1k", 61.0, 10.0),
        ("R16", "100k", 30.0, 10.0),
        ("R17", "100k", 30.0, 17.0),
        ("R18", "200R", 33.0, 18.0),
        ("R19", "5.6k", 33.0, 10.0),
        ("R20", "200k", 33.0, 12.0),
        ("R21", "100k", 33.0, 14.0),
        ("R22", "10k", 34.0, 8.5),
        ("C5", "15pF", 35.0, 20.0),
        ("C6", "15pF", 39.0, 20.0),
        ("C9", "100nF", 33.0, 16.0),
        ("C10", "100nF", 62.0, 18.0),
    ] {
        crate::pcb::rp2040::add_passive(
            &mut board,
            crate::pcb::rp2040::passive(r, value, Point::new(x, y), 0.0),
        );
    }

    // External pack is protected separately: J2.1 positive/J2.2 negative.
    // Q1's intrinsic diode conducts BAT(drain) -> VSYS(source), not the reverse.
    connect(
        &mut board,
        "5V_VBUS",
        &[
            ("J1", "A4"),
            ("U3", "4"),
            ("D1", "2"),
            ("Q1", "1"),
            ("R12", "1"),
            ("C1", "1"),
            ("R1", "2"),
            ("R19", "1"),
        ],
    );
    connect(
        &mut board,
        "VBAT",
        &[("J2", "1"), ("U3", "3"), ("Q1", "3"), ("C7", "1")],
    );
    connect(
        &mut board,
        "VSYS",
        &[
            ("D1", "1"),
            ("Q1", "2"),
            ("U4", "5"),
            ("U4", "8"),
            ("C2", "1"),
            ("R16", "1"),
            ("R20", "1"),
        ],
    );
    connect(
        &mut board,
        "3V3",
        &[
            ("U4", "1"),
            ("U4", "10"),
            ("C3", "1"),
            ("U2", "8"),
            ("J3", "2"),
            ("R3", "1"),
            ("R18", "1"),
            ("Q2", "1"),
            ("LED1", "1"),
            ("C10", "1"),
        ],
    );
    connect(&mut board, "1V1_CORE", &[("U1", "45"), ("C4", "1")]);
    connect(&mut board, "CHG_PROG", &[("U3", "5"), ("R11", "1")]);
    connect(&mut board, "CHG_STAT", &[("U3", "1"), ("LED3", "K")]);
    connect(&mut board, "CHG_LED_ANODE", &[("R1", "1"), ("LED3", "A")]);
    connect(&mut board, "BUCK_LX1", &[("U4", "4"), ("L1", "1")]);
    connect(&mut board, "BUCK_LX2", &[("U4", "2"), ("L1", "2")]);
    connect(&mut board, "3V3_EN", &[("U4", "6"), ("R16", "2")]);
    connect(
        &mut board,
        "BUCK_PS",
        &[("U4", "7"), ("U1", "35"), ("R17", "1")],
    );
    connect(
        &mut board,
        "VBUS_DETECT",
        &[("R19", "2"), ("R22", "1"), ("U1", "36")],
    );
    connect(
        &mut board,
        "VSYS_DIV",
        &[("R20", "2"), ("R21", "1"), ("C9", "1"), ("Q2", "3")],
    );
    // DMG1012T source is the ADC side: its body diode points ADC -> divider.
    // With3V3 disabled this blocks VSYS-divider backfeed into GPIO29.
    connect(&mut board, "VSYS_SENSE", &[("Q2", "2"), ("U1", "41")]);
    // 200R/2.2uF filtered analog reference; AGND header joins the board ground.
    connect(&mut board, "ADC_VREF", &[("R18", "2"), ("C8", "1")]);
    connect(
        &mut board,
        "XTAL_IN",
        &[("U1", "20"), ("Y1", "1"), ("C5", "1")],
    );
    connect(&mut board, "XTAL_OUT", &[("U1", "21"), ("R13", "1")]);
    connect(
        &mut board,
        "XTAL_DRIVE",
        &[("R13", "2"), ("Y1", "3"), ("C6", "1")],
    );
    connect(
        &mut board,
        "RUN_RESET",
        &[("U1", "26"), ("SW1", "2"), ("SW1", "4"), ("R3", "2")],
    );
    connect(
        &mut board,
        "BOOTSEL",
        &[("U1", "56"), ("U2", "1"), ("R14", "1")],
    );
    connect(
        &mut board,
        "BOOT_BUTTON",
        &[("R14", "2"), ("SW2", "2"), ("SW2", "4")],
    );
    connect(&mut board, "QSPI_SCLK", &[("U1", "52"), ("U2", "6")]);
    connect(&mut board, "QSPI_SD0", &[("U1", "53"), ("U2", "5")]);
    connect(&mut board, "QSPI_SD1", &[("U1", "55"), ("U2", "2")]);
    connect(&mut board, "I2C_SDA", &[("U1", "6"), ("J3", "3")]);
    connect(&mut board, "I2C_SCL", &[("U1", "7"), ("J3", "4")]);
    connect(&mut board, "RGB_DATA", &[("U1", "27"), ("LED1", "4")]);
    // LED1.2 DOUT intentionally ends this one-device addressable chain.
    connect(&mut board, "USER_LED", &[("U1", "37"), ("R15", "1")]);
    connect(&mut board, "USER_LED_ANODE", &[("R15", "2"), ("LED2", "A")]);
    connect(&mut board, "SWCLK", &[("U1", "24"), ("J4", "1")]);
    connect(&mut board, "SWDIO", &[("U1", "25"), ("J4", "3")]);
    connect(&mut board, "USB_DP_IN", &[("J1", "A6"), ("R9", "1")]);
    connect(&mut board, "USB_DM_IN", &[("J1", "A7"), ("R10", "1")]);
    connect(&mut board, "USB_DP", &[("R9", "2"), ("U1", "47")]);
    connect(&mut board, "USB_DM", &[("R10", "2"), ("U1", "46")]);
    connect(&mut board, "USB_CC1", &[("J1", "A5"), ("R5", "1")]);
    connect(&mut board, "USB_CC2", &[("J1", "B5"), ("R6", "1")]);
    connect(
        &mut board,
        "GND",
        &[
            ("J2", "2"),
            ("U3", "2"),
            ("U4", "3"),
            ("U4", "9"),
            ("U4", "11"),
            ("J3", "1"),
            ("J3", "MP1"),
            ("J3", "MP2"),
            ("J4", "2"),
            ("U2", "4"),
            ("SW1", "1"),
            ("SW1", "3"),
            ("SW2", "1"),
            ("SW2", "3"),
            ("Y1", "2"),
            ("Y1", "4"),
            ("LED1", "3"),
            ("LED2", "K"),
            ("C1", "2"),
            ("C2", "2"),
            ("C3", "2"),
            ("C4", "2"),
            ("C5", "2"),
            ("C6", "2"),
            ("C7", "2"),
            ("C8", "2"),
            ("C9", "2"),
            ("C10", "2"),
            ("R5", "2"),
            ("R6", "2"),
            ("R11", "2"),
            ("R12", "2"),
            ("R17", "2"),
            ("R21", "2"),
            ("R22", "2"),
        ],
    );

    // Full official header mapping. GPIO23/24/25/29 are internal services, as on Pico.
    let upper = [
        "GPIO0", "GPIO1", "GND", "GPIO2", "GPIO3", "I2C_SDA", "I2C_SCL", "GND", "GPIO6", "GPIO7",
        "GPIO8", "GPIO9", "GND", "GPIO10", "GPIO11", "GPIO12", "GPIO13", "GND", "GPIO14", "GPIO15",
    ];
    let lower = [
        "5V_VBUS",
        "VSYS",
        "GND",
        "3V3_EN",
        "3V3",
        "ADC_VREF",
        "GPIO28",
        "GND",
        "GPIO27",
        "GPIO26",
        "RUN_RESET",
        "GPIO22",
        "GND",
        "GPIO21",
        "GPIO20",
        "GPIO19",
        "GPIO18",
        "GND",
        "GPIO17",
        "RGB_DATA",
    ];
    for (reference, names) in [("HDR1", upper), ("HDR2", lower)] {
        for (i, name) in names.iter().enumerate() {
            connect(&mut board, name, &[(reference, &(i + 1).to_string())]);
        }
    }
    for (gpio, pin) in [
        (0, 2),
        (1, 3),
        (2, 4),
        (3, 5),
        (6, 8),
        (7, 9),
        (8, 11),
        (9, 12),
        (10, 13),
        (11, 14),
        (12, 15),
        (13, 16),
        (14, 17),
        (15, 18),
        (17, 28),
        (18, 29),
        (19, 30),
        (20, 31),
        (21, 32),
        (22, 34),
        (26, 38),
        (27, 39),
        (28, 40),
    ] {
        connect(
            &mut board,
            &format!("GPIO{gpio}"),
            &[("U1", &pin.to_string())],
        );
    }
    crate::pcb::rp2040::complete_core(&mut board);
    // Native pin-role placement: orient the flash bank, leave the bypass body
    // clear, and place the ADC filter and core capacitor at their actual pins.
    // This body-valid draft still has unfinished routes; see experiment32.
    board.find_footprint_mut("U1").unwrap().rotation = 90.0;
    for (reference, x, y) in [
        ("C4", 36.55, 9.6),
        ("R18", 46.0, 18.5),
        ("C27", 46.0, 17.5),
        ("C29", 46.0, 14.5),
    ] {
        board.find_footprint_mut(reference).unwrap().position = Point::new(x, y);
    }
    // The real JST mounting body requires a rigid LED/bypass translation.
    // Keep both actual capacitor-to-LED pin distances and the clock bank intact.
    for (reference, x, y) in [("LED1", 59.805, 15.0), ("C10", 59.805, 18.25)] {
        board.find_footprint_mut(reference).unwrap().position = Point::new(x, y);
    }
    board
}

pub fn solve_pico_ultimate_board(config: &RouterConfig) -> Result<PcbBoard, RoutingError> {
    let mut board = create_pico_ultimate_board();
    route_board(&mut board, config)?;
    Ok(board)
}

/// Includes every actual part. Exact MPNs are added only for reviewed sources;
/// remaining entries retain the existing engineering sourcing/land-pattern hold.
pub fn generate_pico_ultimate_bom() -> Vec<BomEntry> {
    generate_pico_ultimate_bom_for_board(&create_pico_ultimate_board())
}

/// Preserve reviewed sourcing only for unchanged actual value/package pairs.
pub fn generate_pico_ultimate_bom_for_board(board: &PcbBoard) -> Vec<BomEntry> {
    let mut bom = generate_bom_from_board(board);
    for row in &mut bom {
        let sourced = match row.designator.as_str() {
            "U3" if row.value == "MCP73831T-2ACI/OT" && row.package == "SOT-23-5" => {
                Some(("Microchip", "MCP73831T-2ACI/OT"))
            }
            "U4" if row.value == "RT6150B-33GQW"
                && row.package == "WDFN-10-1EP_2.5x2.5mm_P0.5mm_EP1.2x2.0mm" =>
            {
                Some(("Richtek", "RT6150B-33GQW"))
            }
            "Q1" if row.value == "DMG2305UX-7" && row.package == "SOT-23" => {
                Some(("Diodes Incorporated", "DMG2305UX-7"))
            }
            "Q2" if row.value == "DMG1012T-7" && row.package == "SOT-523" => {
                Some(("Diodes Incorporated", "DMG1012T-7"))
            }
            "D1" if row.value == "MBR120VLSFT1G" && row.package == "D_SOD-123" => {
                Some(("onsemi", "MBR120VLSFT1G"))
            }
            "L1" if row.value == "2.2uH" && row.package == "L_TDK_VLS252012HBX_2.5x2.0mm" => {
                Some(("TDK", "VLS252012HBX-2R2M-1"))
            }
            "Y1" if row.value == "ABM8-272-T3 12MHz" && row.package == "Crystal_SMD_3225-4Pin" => {
                Some(("Abracon", "ABM8-272-T3"))
            }
            "LED1" if row.value == "WS2812B-V6" && row.package == "LED_WS2812B-V6_5050" => {
                Some(("Worldsemi", "WS2812B-V6"))
            }
            _ => None,
        };
        if let Some((manufacturer, mpn)) = sourced {
            row.manufacturer = manufacturer.into();
            row.mpn = mpn.into();
        }
    }
    bom
}

pub fn generate_pico_ultimate_cpl() -> Vec<CplEntry> {
    generate_cpl_from_board(&create_pico_ultimate_board())
}

#[cfg(test)]
mod placement_tests {
    use super::*;

    #[test]
    fn pico_draft_has_clear_bodies_and_local_adc_bypass() {
        let board = create_pico_ultimate_board();
        assert!(crate::pcb::placement_issues(&board).is_empty());
        let pin = |reference: &str, number: &str| {
            let fp = board.find_footprint(reference).unwrap();
            let pad = fp.pads.iter().find(|p| p.pin_number == number).unwrap();
            (fp.pad_world_position(pad), pad.net)
        };
        let (cap, cap_net) = pin("C27", "1");
        let (adc, adc_net) = pin("U1", "43");
        assert_eq!(cap_net, adc_net);
        assert_eq!(board.get_net_name(cap_net.unwrap()), Some("ADC_VREF"));
        assert!(((cap.x-adc.x).powi(2)+(cap.y-adc.y).powi(2)).sqrt() < 1.1);
        assert_eq!((board.width, board.height), (72.0, 30.0));
        assert!(board.tracks.is_empty() && board.vias.is_empty());
    }
}
