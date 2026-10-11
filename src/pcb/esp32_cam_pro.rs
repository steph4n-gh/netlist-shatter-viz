//! Source-backed ESP32 camera engineering circuit, self-powered from EXT_5V.
//! Sources/maps: build/esp32-cam/circuit-audit-01. No USB-only full-load claim.
//! Exact camera mating, regulator thermals, antenna, ESD and physical qualification
//! remain holds; connected/routed copper cannot prove them.
use crate::pcb::assembly::{BomEntry, CplEntry, generate_bom_from_board, generate_cpl_from_board};
use crate::pcb::core::{PcbBoard, Point};
use crate::pcb::footprint::*;
use crate::pcb::router::{RouterConfig, RoutingError, route_board};

fn connect(board: &mut PcbBoard, name: &str, pins: &[(&str, &str)]) {
    for &(reference, pin) in pins {
        board
            .assign_pad_net(reference, pin, name)
            .expect("Invalid ESP32 camera netlist");
    }
}

/// Actual firmware pin map, matching camera physical connector Y2..Y9 ordering.
pub const CAMERA_DATA_GPIOS: [u8; 8] = [5, 18, 19, 21, 36, 39, 34, 35];
pub const CAMERA_XCLK_GPIO: u8 = 0;
pub const CAMERA_PWDN_GPIO: u8 = 32;
pub const SD_SPI_GPIOS: [u8; 4] = [12, 13, 33, 15]; // CLK,MISO,MOSI,CS
pub const EXTERNAL_5V_SUPPLY_MIN_MA: u32 = 1000;
pub const DECLARED_TOTAL_ALLOCATION_MA: u32 = 800;
/// These holds are intentionally not waived by a successful routing result.
pub const QUALIFICATION_HOLDS: &[&str] = &[
    "External regulated5V4.75..5.25V>=1A required; USB VBUS sensing only",
    "800mA declared allocation is not verified worst-case camera/card demand",
    "AMS1117 manufacturer-specific capacitor ESR/stability and about1.48W thermal dissipation held",
    "Exact Ai-Thinker OV2640 cable/contact face, connector mating and mechanical fit held",
    "WROVER antenna keepout/placement and RF tuning not enforced by current board model",
    "USB connector NPTH/slots, USB differential return path/ESD and self-powered provisioning held",
    "Exact LED/switch/USB protection sourcing, native lands/mask/stencil and physical operation held",
];

/// Complete connected engineering netlist. Dual headers remain shared GPIO/power,
/// not independent GPIOs. SD uses GPIO-matrix SPI instead of strap-conflicting SDMMC.
pub fn create_esp32_cam_pro_board() -> PcbBoard {
    let mut b = PcbBoard::new(94.0, 32.0);
    for f in [
        usb_c_16pin("J1", "USB-C_UNQUALIFIED_LAND")
            .at_position(Point::new(4.5, 16.0))
            .with_rotation(270.0),
        cp2102n_qfn28("U4", "CP2102N-A02-GQFN28").at_position(Point::new(17.0, 16.0)),
        sot23("Q1", "MMBT3904").at_position(Point::new(22.5, 10.5)),
        sot23("Q2", "MMBT3904").at_position(Point::new(22.5, 21.5)),
        sot23("Q3", "MMBT3904").at_position(Point::new(87.0, 8.5)),
        sot223("U2", "AMS1117-3.3_THERMAL_HOLD").at_position(Point::new(28.0, 16.0)),
        sot23_5("U3", "LP5907MFX-2.8/NOPB").at_position(Point::new(32.0, 10.0)),
        sot23_5("U5", "LP5907MFX-1.2/NOPB").at_position(Point::new(32.0, 22.0)),
        esp32_wrover_e("U1", "ESP32-WROVER-E-N4R8")
            .at_position(Point::new(53.0, 16.0))
            .with_rotation(90.0),
        ov2640_hirose_fh12("J2", "AiThinker_OV2640_FPC24")
            .at_position(Point::new(76.5, 16.0))
            .with_rotation(90.0),
        microsd_molex_104031("J3", "104031-0811")
            .at_position(Point::new(86.0, 17.0))
            .with_rotation(90.0),
        // Bounded automatic placement separates switch envelopes from U1;
        // measured cold app route: build/esp32-cam/complete-delivery-05.
        pts636_smd("SW1", "RESET").at_position(Point::new(66.25, 3.75)),
        pts636_smd("SW2", "BOOT").at_position(Point::new(70.0, 27.5)),
        led_0805("LED1", "WHITE_LED_SOURCING_HOLD").at_position(Point::new(88.0, 5.0)),
        led_0805("LED2", "GREEN_LED_SOURCING_HOLD").at_position(Point::new(88.0, 27.0)),
        header_1x16("HDR1", "SHARED_SIGNALS_EXT5V_INPUT")
            .at_position(Point::new(18.0, 2.54))
            .with_rotation(270.0),
        header_1x16("HDR2", "SHARED_CAMERA_SIGNALS")
            .at_position(Point::new(18.0, 29.46))
            .with_rotation(270.0),
    ] {
        b.add_footprint(f);
    }
    // Keep transistor semantic pin names honest without changing shared SOT23 helper.
    for reference in ["Q1", "Q2", "Q3"] {
        let f = b.find_footprint_mut(reference).unwrap();
        for (pin, name) in [("1", "B"), ("2", "E"), ("3", "C")] {
            f.pad_mut(pin).unwrap().pin_name = name.into();
        }
    }
    for reference in ["U3", "U5"] {
        let f = b.find_footprint_mut(reference).unwrap();
        for (pin, name) in [
            ("1", "IN"),
            ("2", "GND"),
            ("3", "EN"),
            ("4", "NC"),
            ("5", "OUT"),
        ] {
            f.pad_mut(pin).unwrap().pin_name = name.into();
        }
    }
    // Supply capacitors retain larger0805 bodies; placement helper avoids fixed envelopes.
    for (r, v, x, y) in [
        ("C1", "10uF", 28.0, 8.0),
        ("C2", "22uF_ESR_HOLD", 28.0, 24.0),
        ("C3", "1uF", 32.0, 7.0),
        ("C4", "1uF", 35.0, 10.0),
        ("C5", "100nF", 74.0, 8.0),
        ("C6", "1uF", 32.0, 25.0),
        ("C7", "1uF", 35.0, 22.0),
        ("C8", "100nF", 74.0, 24.0),
        ("C9", "10uF", 71.0, 8.0),
        ("C10", "100nF", 71.0, 24.0),
        ("C11", "4.7uF", 17.0, 11.0),
        ("C12", "100nF", 17.0, 12.0),
        ("C13", "4.7uF", 17.0, 21.0),
        ("C14", "100nF", 17.0, 20.0),
        ("C15", "1uF", 38.0, 6.0),
        ("C16", "100nF", 73.0, 6.0),
        ("C17", "10uF", 85.0, 24.0),
        ("C18", "100nF", 85.0, 25.0),
        ("C19", "10uF", 53.0, 6.0),
        ("C20", "100nF", 53.0, 26.0),
    ] {
        crate::pcb::rp2040::add_passive(&mut b, capacitor_0805(r, v).at_position(Point::new(x, y)));
    }
    // Real pullups/straps/base resistors and LED limits; no disconnected placeholders.
    for (r, v, x, y) in [
        ("R1", "10k", 38.0, 8.0),
        ("R2", "10k", 38.0, 24.0),
        ("R3", "10k", 22.5, 6.0),
        ("R4", "10k", 22.5, 26.0),
        ("R5", "5.1k", 11.5, 10.5),
        ("R6", "5.1k", 11.5, 21.5),
        ("R7", "22.1k", 11.0, 14.0),
        ("R8", "47.5k", 11.0, 18.0),
        ("R9", "1k", 18.0, 8.0),
        ("R10", "4.7k", 74.0, 9.0),
        ("R11", "4.7k", 74.0, 23.0),
        ("R12", "4.7k", 73.75, 6.0),
        ("R13", "1k", 73.0, 30.0),
        ("R14", "10k", 39.0, 5.0),
        ("R15", "10k", 39.0, 27.0),
        ("R16", "10k", 82.0, 6.0),
        ("R17", "10k", 82.0, 8.0),
        ("R18", "10k", 82.0, 24.0),
        ("R19", "10k", 82.0, 26.0),
        ("R20", "10k", 82.0, 28.0),
        ("R21", "1k", 86.0, 6.0),
        ("R22", "100k", 86.0, 10.0),
        ("R23", "330R", 90.0, 6.0),
        ("R24", "1k", 90.0, 26.0),
    ] {
        crate::pcb::rp2040::add_passive(
            &mut b,
            crate::pcb::rp2040::passive(r, v, Point::new(x, y), 0.0),
        );
    }

    connect(
        &mut b,
        "GND",
        &[
            ("J1", "A1"),
            ("J1", "B1"),
            ("J1", "SH1"),
            ("J1", "SH2"),
            ("J1", "SH3"),
            ("J1", "SH4"),
            ("U1", "1"),
            ("U1", "15"),
            ("U1", "38"),
            ("U2", "1"),
            ("U3", "2"),
            ("U5", "2"),
            ("U4", "3"),
            ("U4", "29"),
            ("U4", "23"),
            ("J2", "10"),
            ("J2", "23"),
            ("J2", "MP1"),
            ("J2", "MP2"),
            ("J3", "6"),
            ("J3", "SH1"),
            ("J3", "SH2"),
            ("J3", "SH3"),
            ("J3", "SH4"),
            ("SW1", "1"),
            ("SW2", "1"),
            ("Q3", "2"),
            ("LED2", "1"),
            ("HDR1", "2"),
            ("HDR1", "4"),
            ("HDR2", "2"),
            ("R8", "2"),
            ("R13", "2"),
            ("R14", "2"),
            ("R15", "2"),
            ("R22", "2"),
        ],
    );
    for i in 1..=9 {
        connect(&mut b, "GND", &[("U1", &format!("EP{i}"))]);
    }
    for i in 1..=20 {
        connect(&mut b, "GND", &[(&format!("C{i}"), "2")]);
    }
    // VBUS never reaches EXT5V: deliberately self-powered, no host backfeed.
    connect(
        &mut b,
        "USB_VBUS",
        &[("J1", "A4"), ("J1", "B4"), ("R7", "1")],
    );
    connect(
        &mut b,
        "VBUS_SENSE",
        &[("R7", "2"), ("R8", "1"), ("U4", "8")],
    );
    connect(
        &mut b,
        "EXT_5V",
        &[
            ("HDR1", "1"),
            ("U2", "3"),
            ("C1", "1"),
            ("U4", "7"),
            ("C11", "1"),
            ("C12", "1"),
            ("R23", "1"),
        ],
    );
    connect(
        &mut b,
        "3V3_SYS",
        &[
            ("U2", "2"),
            ("U2", "4"),
            ("U1", "2"),
            ("U3", "1"),
            ("U3", "3"),
            ("U5", "1"),
            ("U5", "3"),
            ("J2", "14"),
            ("J3", "4"),
            ("HDR2", "1"),
            ("C2", "1"),
            ("C3", "1"),
            ("C6", "1"),
            ("C9", "1"),
            ("C10", "1"),
            ("C17", "1"),
            ("C18", "1"),
            ("C19", "1"),
            ("C20", "1"),
            ("R1", "1"),
            ("R2", "1"),
            ("R10", "1"),
            ("R11", "1"),
            ("R12", "1"),
            ("R16", "1"),
            ("R17", "1"),
            ("R18", "1"),
            ("R19", "1"),
            ("R20", "1"),
            ("R24", "1"),
        ],
    );
    connect(
        &mut b,
        "USB_3V3",
        &[("U4", "6"), ("R9", "1"), ("C13", "1"), ("C14", "1")],
    );
    connect(&mut b, "BRIDGE_RESET_N", &[("U4", "9"), ("R9", "2")]);
    connect(
        &mut b,
        "CAM_AVDD",
        &[
            ("U3", "5"),
            ("C4", "1"),
            ("C5", "1"),
            ("J2", "21"),
            ("HDR2", "3"),
        ],
    );
    connect(
        &mut b,
        "CAM_DVDD",
        &[
            ("U5", "5"),
            ("C7", "1"),
            ("C8", "1"),
            ("J2", "15"),
            ("HDR2", "4"),
        ],
    );
    connect(&mut b, "USB_DP", &[("J1", "A6"), ("J1", "B6"), ("U4", "4")]);
    connect(&mut b, "USB_DN", &[("J1", "A7"), ("J1", "B7"), ("U4", "5")]);
    connect(&mut b, "USB_CC1", &[("J1", "A5"), ("R5", "1")]);
    connect(&mut b, "USB_CC2", &[("J1", "B5"), ("R6", "1")]);
    connect(&mut b, "GND", &[("R5", "2"), ("R6", "2")]);
    connect(
        &mut b,
        "UART_RX0",
        &[("U1", "34"), ("U4", "26"), ("HDR1", "5")],
    );
    connect(
        &mut b,
        "UART_TX0",
        &[("U1", "35"), ("U4", "25"), ("HDR1", "6")],
    );
    connect(
        &mut b,
        "UART_DTR",
        &[("U4", "28"), ("R3", "1"), ("Q2", "2")],
    );
    connect(
        &mut b,
        "UART_RTS",
        &[("U4", "24"), ("R4", "1"), ("Q1", "2")],
    );
    connect(&mut b, "DTR_BASE", &[("R3", "2"), ("Q1", "1")]);
    connect(&mut b, "RTS_BASE", &[("R4", "2"), ("Q2", "1")]);
    connect(
        &mut b,
        "ESP_EN",
        &[
            ("U1", "3"),
            ("Q1", "3"),
            ("SW1", "2"),
            ("R1", "2"),
            ("C15", "1"),
            ("HDR1", "3"),
        ],
    );
    connect(
        &mut b,
        "ESP_IO0",
        &[
            ("U1", "25"),
            ("J2", "12"),
            ("Q2", "3"),
            ("SW2", "2"),
            ("R2", "2"),
            ("HDR1", "7"),
        ],
    );
    connect(
        &mut b,
        "CAM_RESET_N",
        &[("J2", "19"), ("R12", "2"), ("C16", "1")],
    );
    connect(
        &mut b,
        "CAM_PWDN",
        &[("J2", "17"), ("U1", "8"), ("R13", "1"), ("HDR1", "15")],
    );
    connect(
        &mut b,
        "GPIO2",
        &[("U1", "24"), ("R14", "1"), ("HDR1", "8")],
    );
    connect(
        &mut b,
        "SD_CLK",
        &[("U1", "14"), ("J3", "5"), ("R15", "1"), ("HDR1", "10")],
    );
    connect(
        &mut b,
        "SD_MISO",
        &[("U1", "16"), ("J3", "7"), ("R16", "2"), ("HDR1", "11")],
    );
    connect(
        &mut b,
        "SD_MOSI",
        &[("U1", "9"), ("J3", "3"), ("R17", "2"), ("HDR1", "12")],
    );
    connect(
        &mut b,
        "SD_CS",
        &[("U1", "23"), ("J3", "2"), ("R18", "2"), ("HDR1", "13")],
    );
    connect(&mut b, "SD_DAT1_UNUSED", &[("J3", "8"), ("R19", "2")]);
    connect(&mut b, "SD_DAT2_UNUSED", &[("J3", "1"), ("R20", "2")]);
    connect(&mut b, "GPIO14", &[("U1", "13"), ("HDR1", "14")]);
    connect(
        &mut b,
        "FLASH_GPIO4",
        &[("U1", "26"), ("R21", "1"), ("HDR1", "9")],
    );
    connect(
        &mut b,
        "FLASH_BASE",
        &[("R21", "2"), ("Q3", "1"), ("R22", "1")],
    );
    connect(&mut b, "FLASH_ANODE", &[("R23", "2"), ("LED1", "2")]);
    connect(&mut b, "FLASH_CATHODE", &[("LED1", "1"), ("Q3", "3")]);
    connect(&mut b, "PWR_LED_ANODE", &[("R24", "2"), ("LED2", "2")]);
    for (name, mcu, fpc, header) in [
        ("CAM_D0", "29", "6", "5"),
        ("CAM_D1", "30", "4", "6"),
        ("CAM_D2", "31", "3", "7"),
        ("CAM_D3", "33", "5", "8"),
        ("CAM_D4", "4", "7", "9"),
        ("CAM_D5", "5", "9", "10"),
        ("CAM_D6", "6", "11", "11"),
        ("CAM_D7", "7", "13", "12"),
        ("CAM_VSYNC", "10", "18", "13"),
        ("CAM_HREF", "37", "16", "14"),
        ("CAM_SIOC", "12", "20", "15"),
        ("CAM_SIOD", "11", "22", "16"),
    ] {
        connect(&mut b, name, &[("U1", mcu), ("J2", fpc), ("HDR2", header)]);
    }
    connect(&mut b, "CAM_SIOC", &[("R10", "2")]);
    connect(&mut b, "CAM_SIOD", &[("R11", "2")]);
    connect(
        &mut b,
        "CAM_PCLK",
        &[("U1", "36"), ("J2", "8"), ("HDR1", "16")],
    );
    b
}

pub fn solve_esp32_cam_pro_board(config: &RouterConfig) -> Result<PcbBoard, RoutingError> {
    let mut board = create_esp32_cam_pro_board();
    route_board(&mut board, config)?;
    Ok(board)
}
/// Exact actual parts; source-backed MPNs only when actual value/package still match.
pub fn generate_esp32_cam_pro_bom() -> Vec<BomEntry> {
    generate_esp32_cam_pro_bom_for_board(&create_esp32_cam_pro_board())
}
pub fn generate_esp32_cam_pro_bom_for_board(board: &PcbBoard) -> Vec<BomEntry> {
    let mut bom = generate_bom_from_board(board);
    for row in &mut bom {
        let sourced = match (row.value.as_str(), row.package.as_str()) {
            ("ESP32-WROVER-E-N4R8", "ESP32-WROVER-E_18x31.4mm") => {
                Some(("Espressif Systems", "ESP32-WROVER-E-N4R8"))
            }
            ("CP2102N-A02-GQFN28", "CP2102N_QFN28_5x5mm_P0.5mm_EP3.35") => {
                Some(("Silicon Labs", "CP2102N-A02-GQFN28"))
            }
            ("LP5907MFX-2.8/NOPB", "SOT-23-5") => Some(("Texas Instruments", "LP5907MFX-2.8/NOPB")),
            ("LP5907MFX-1.2/NOPB", "SOT-23-5") => Some(("Texas Instruments", "LP5907MFX-1.2/NOPB")),
            _ => None,
        };
        if let Some((m, p)) = sourced {
            row.manufacturer = m.into();
            row.mpn = p.into();
            row.description =
                "Source-backed circuit part; assembly/physical qualification remains held".into();
        }
    }
    bom
}
pub fn generate_esp32_cam_pro_cpl() -> Vec<CplEntry> {
    generate_cpl_from_board(&create_esp32_cam_pro_board())
}
