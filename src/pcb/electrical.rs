//! Electrical assertions separate from copper DRC. A routed wrong netlist is still wrong.
use super::*;
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct ElectricalIssue {
    pub component: String,
    pub message: String,
}

fn pin_net(f: &Footprint, pin: &str) -> Option<NetId> {
    f.pads
        .iter()
        .find(|p| p.pin_number == pin)
        .and_then(|p| p.net)
}

fn resistor_between(
    board: &PcbBoard,
    a: Option<NetId>,
    b: Option<NetId>,
    value: Option<&str>,
) -> bool {
    a.is_some()
        && b.is_some()
        && a != b
        && board.footprints.iter().any(|r| {
            r.reference.starts_with('R')
                && value.is_none_or(|v| r.value == v)
                && ((pin_net(r, "1") == a && pin_net(r, "2") == b)
                    || (pin_net(r, "1") == b && pin_net(r, "2") == a))
        })
}

pub fn check_electrical(board: &PcbBoard) -> Vec<ElectricalIssue> {
    let mut issues = Vec::new();
    for f in &board.footprints {
        let mut require = |pin: &str, expected: &[&str]| {
            let name = f
                .pads
                .iter()
                .find(|p| p.pin_number == pin)
                .and_then(|p| p.net)
                .and_then(|n| board.get_net_name(n));
            if !name.is_some_and(|n| expected.contains(&n)) {
                issues.push(ElectricalIssue {
                    component: f.reference.clone(),
                    message: format!(
                        "pin {pin}: expected {}, found {}",
                        expected.join(" or "),
                        name.unwrap_or("unconnected")
                    ),
                });
            }
        };
        if f.value.starts_with("RP2040") {
            for p in ["1", "10", "22", "33", "42", "44", "48", "49"] {
                require(p, &["3V3"]);
            }
            if board.net_names.values().any(|name| name == "ADC_VREF") {
                require("43", &["ADC_VREF"]);
            } else {
                require("43", &["3V3"]);
            }
            for p in ["23", "45", "50"] {
                require(p, &["1V1_CORE"]);
            }
            for p in ["19", "57"] {
                require(p, &["GND"]);
            }
            require("47", &["USB_DP"]);
            require("46", &["USB_DM"]);
            require("52", &["QSPI_CLK", "QSPI_SCLK"]);
            require("53", &["QSPI_D0", "QSPI_SD0"]);
            require("55", &["QSPI_D1", "QSPI_SD1"]);
            require("54", &["QSPI_D2"]);
            require("51", &["QSPI_D3"]);
            require("56", &["QSPI_CS", "BOOTSEL"]);
            require("26", &["RUN_RESET"]);
        }
        if f.package == "USB_C_Receptacle_16Pin" {
            for p in ["A1", "B1", "SH1", "SH2", "SH3", "SH4"] {
                require(p, &["GND"]);
            }
            for (a, b) in [("A4", "B4"), ("A6", "B6"), ("A7", "B7")] {
                let net = |p: &str| {
                    f.pads
                        .iter()
                        .find(|x| x.pin_number == p)
                        .and_then(|x| x.net)
                };
                if net(a).is_none() || net(a) != net(b) {
                    issues.push(ElectricalIssue {
                        component: f.reference.clone(),
                        message: format!("USB contacts {a}/{b} must share a connected net"),
                    });
                }
            }
        }
        if f.value.starts_with("RP2040") {
            let gnd = board
                .net_names
                .iter()
                .find(|(_, name)| name.as_str() == "GND")
                .map(|(n, _)| *n);
            let vdd = board
                .net_names
                .iter()
                .find(|(_, name)| name.as_str() == "3V3")
                .map(|(n, _)| *n);
            let adc = pin_net(f, "43");
            if adc != vdd {
                let filtered = resistor_between(board, vdd, adc, Some("200R"));
                let bypassed = adc.is_some()
                    && gnd.is_some()
                    && board.footprints.iter().any(|c| {
                        c.reference.starts_with('C')
                            && c.value == "2.2uF"
                            && ((pin_net(c, "1") == adc && pin_net(c, "2") == gnd)
                                || (pin_net(c, "2") == adc && pin_net(c, "1") == gnd))
                    });
                if !filtered || !bypassed {
                    issues.push(ElectricalIssue {component:f.reference.clone(), message:"ADC_VREF requires the specified 200R supply filter and grounded 2.2uF bypass".into()});
                }
            }
            if !resistor_between(board, pin_net(f, "26"), vdd, Some("10k")) {
                issues.push(ElectricalIssue {
                    component: f.reference.clone(),
                    message: "RUN lacks the specified 10k pull-up to 3V3".into(),
                });
            }
            for crystal in board
                .footprints
                .iter()
                .filter(|c| c.package == "Crystal_SMD_3225-4Pin")
            {
                if pin_net(f, "20").is_none() || pin_net(f, "20") != pin_net(crystal, "1") {
                    issues.push(ElectricalIssue {
                        component: crystal.reference.clone(),
                        message: "Crystal input must connect to RP2040 XIN".into(),
                    });
                }
                if !resistor_between(board, pin_net(f, "21"), pin_net(crystal, "3"), Some("1k")) {
                    issues.push(ElectricalIssue {component:crystal.reference.clone(), message:"Crystal drive lacks the specified 1k XOUT series resistor for 3.3V IOVDD".into()});
                }
            }
            for switch in board.footprints.iter().filter(|s| s.value == "BOOTSEL") {
                if gnd.is_none()
                    || pin_net(switch, "1") != gnd
                    || !resistor_between(board, pin_net(f, "56"), pin_net(switch, "2"), Some("1k"))
                {
                    issues.push(ElectricalIssue {component:switch.reference.clone(), message:"BOOTSEL must ground QSPI CS through the specified 1k isolation resistor".into()});
                }
            }
            for led in board
                .footprints
                .iter()
                .filter(|d| d.package == "LED_0805_2012Metric")
            {
                // A charger STAT indicator has a different current path from a GPIO LED.
                // Its VBUS-fed series resistor is checked with the actual charger below.
                if board.footprints.iter().any(|c| {
                    c.value.starts_with("MCP73831")
                        && pin_net(c, "1").is_some()
                        && pin_net(c, "1") == pin_net(led, "1")
                }) {
                    continue;
                }
                let series_from_mcu = f.pads.iter().any(|p| {
                    p.net != gnd
                        && p.net != vdd
                        && resistor_between(board, p.net, pin_net(led, "2"), Some("1k"))
                });
                if gnd.is_none() || pin_net(led, "1") != gnd || !series_from_mcu {
                    issues.push(ElectricalIssue {component:led.reference.clone(), message:"Status LED cathode must be grounded and anode driven through the specified 1k series resistor".into()});
                }
            }
        }
        if f.package == "Crystal_SMD_3225-4Pin"
            && (f.value.contains("12MHz") || f.value.starts_with("ABM8"))
        {
            for pin in ["1", "3"] {
                let net = f
                    .pads
                    .iter()
                    .find(|p| p.pin_number == pin)
                    .and_then(|p| p.net);
                let has_load = net.is_some_and(|n| {
                    board.footprints.iter().any(|c| {
                        c.reference.starts_with('C')
                            && c.value.contains("pF")
                            && c.pads.iter().any(|p| p.net == Some(n))
                            && c.pads
                                .iter()
                                .any(|p| p.net.and_then(|n| board.get_net_name(n)) == Some("GND"))
                    })
                });
                if !has_load {
                    issues.push(ElectricalIssue {
                        component: f.reference.clone(),
                        message: format!(
                            "Crystal pin {pin} lacks a specified grounded load capacitor"
                        ),
                    });
                }
            }
        }
        // These legacy designs require complete circuit/part validation, not inference from geometry.
        if f.value.contains("GW1N") {
            issues.push(ElectricalIssue {component:f.reference.clone(),message:"M64 prototype: FPGA package/pinout, complete console bus, configuration firmware and power rails are not validated".into()});
        }
        if f.value == "CH340N" {
            issues.push(ElectricalIssue {component:f.reference.clone(),message:"CH340N cannot implement the declared DTR+RTS auto-reset interface; redesign the USB bridge and camera data bus before fabrication".into()});
        }
        if f.value.starts_with("MCP73831") {
            let net = |name: &str| {
                board
                    .net_names
                    .iter()
                    .find(|(_, n)| n.as_str() == name)
                    .map(|(id, _)| *id)
            };
            let gnd = net("GND");
            let vbus = net("5V_VBUS");
            let vbat = net("VBAT");
            let vsys = net("VSYS");
            if gnd.is_none()
                || vbus.is_none()
                || vbat.is_none()
                || pin_net(f, "2") != gnd
                || pin_net(f, "3") != vbat
                || pin_net(f, "4") != vbus
            {
                issues.push(ElectricalIssue {
                    component: f.reference.clone(),
                    message: "Charger VSS/VBAT/VDD must connect to GND/VBAT/5V_VBUS".into(),
                });
            }
            if !resistor_between(board, pin_net(f, "5"), gnd, Some("10k")) {
                issues.push(ElectricalIssue {component:f.reference.clone(),message:"Charger PROG must use the specified 10k resistor for nominal 100mA charging".into()});
            }
            let stat = pin_net(f, "1");
            let status_ok = stat.is_some()
                && board.footprints.iter().any(|led| {
                    led.package == "LED_0805_2012Metric"
                        && pin_net(led, "1") == stat
                        && resistor_between(board, pin_net(led, "2"), vbus, Some("2.0k"))
                });
            if !status_ok {
                issues.push(ElectricalIssue {
                    component: f.reference.clone(),
                    message:
                        "Charger STAT LED must be fed from VBUS through its specified 2.0k resistor"
                            .into(),
                });
            }
            let load_switch = board.footprints.iter().find(|q| q.value == "DMG2305UX-7");
            if !load_switch.is_some_and(|q| {
                vbus.is_some()
                    && vsys.is_some()
                    && vbat.is_some()
                    && pin_net(q, "1") == vbus
                    && pin_net(q, "2") == vsys
                    && pin_net(q, "3") == vbat
                    && resistor_between(board, vbus, gnd, Some("100k"))
            }) {
                issues.push(ElectricalIssue {component:f.reference.clone(),message:"Battery load switch must use documented gate=VBUS/source=VSYS/drain=VBAT orientation and 100k gate pull-down".into()});
            }
            if !board.footprints.iter().any(|d| {
                d.value == "MBR120VLSFT1G"
                    && vbus.is_some()
                    && vsys.is_some()
                    && pin_net(d, "1") == vsys
                    && pin_net(d, "2") == vbus
            }) {
                issues.push(ElectricalIssue {
                    component: f.reference.clone(),
                    message: "USB power diode must have anode on VBUS and cathode on VSYS".into(),
                });
            }
            if !board.footprints.iter().any(|r| r.value == "RT6150B-33GQW") {
                issues.push(ElectricalIssue {component:f.reference.clone(),message:"Declared 1S battery circuit requires the specified buck-boost regulator, not the obsolete LDO path".into()});
            }
            for (rail, name) in [(vbus, "VBUS"), (vbat, "VBAT")] {
                if rail.is_none()
                    || gnd.is_none()
                    || !board.footprints.iter().any(|c| {
                        c.reference.starts_with('C')
                            && c.value == "10uF"
                            && ((pin_net(c, "1") == rail && pin_net(c, "2") == gnd)
                                || (pin_net(c, "2") == rail && pin_net(c, "1") == gnd))
                    })
                {
                    issues.push(ElectricalIssue {
                        component: f.reference.clone(),
                        message: format!(
                            "Charger {name} requires its specified grounded 10uF capacitor"
                        ),
                    });
                }
            }
        }
        if f.value == "RT6150B-33GQW" {
            let net = |name: &str| {
                board
                    .net_names
                    .iter()
                    .find(|(_, n)| n.as_str() == name)
                    .map(|(id, _)| *id)
            };
            let gnd = net("GND");
            let vsys = net("VSYS");
            let vdd = net("3V3");
            let rails_ok = gnd.is_some()
                && vsys.is_some()
                && vdd.is_some()
                && ["3", "9", "11"].iter().all(|p| pin_net(f, p) == gnd)
                && ["5", "8"].iter().all(|p| pin_net(f, p) == vsys)
                && ["1", "10"].iter().all(|p| pin_net(f, p) == vdd);
            if !rails_ok {
                issues.push(ElectricalIssue {component:f.reference.clone(),message:"Buck-boost VIN/VINA, VOUT/FB and all grounds/exposed pad must match the fixed RT6150B-33 pinout".into()});
            }
            let lx1 = pin_net(f, "4");
            let lx2 = pin_net(f, "2");
            if lx1.is_none()
                || lx2.is_none()
                || lx1 == lx2
                || !board.footprints.iter().any(|l| {
                    l.reference.starts_with('L')
                        && l.value == "2.2uH"
                        && ((pin_net(l, "1") == lx1 && pin_net(l, "2") == lx2)
                            || (pin_net(l, "2") == lx1 && pin_net(l, "1") == lx2))
                })
            {
                issues.push(ElectricalIssue {
                    component: f.reference.clone(),
                    message: "Buck-boost requires the specified 2.2uH inductor between LX1 and LX2"
                        .into(),
                });
            }
            if !resistor_between(board, pin_net(f, "6"), vsys, Some("100k")) {
                issues.push(ElectricalIssue {
                    component: f.reference.clone(),
                    message: "Buck-boost EN requires its 100k pull-up to VSYS".into(),
                });
            }
            let mode = pin_net(f, "7");
            if !resistor_between(board, mode, gnd, Some("100k"))
                || !board.footprints.iter().any(|m| {
                    m.value.starts_with("RP2040") && mode.is_some() && pin_net(m, "35") == mode
                })
            {
                issues.push(ElectricalIssue {
                    component: f.reference.clone(),
                    message:
                        "Buck-boost PS requires the specified pull-down and GPIO23 mode connection"
                            .into(),
                });
            }
            for (rail, name, value) in [(vsys, "VSYS", "10uF"), (vdd, "3V3", "47uF")] {
                if rail.is_none()
                    || gnd.is_none()
                    || !board.footprints.iter().any(|c| {
                        c.reference.starts_with('C')
                            && c.value == value
                            && ((pin_net(c, "1") == rail && pin_net(c, "2") == gnd)
                                || (pin_net(c, "2") == rail && pin_net(c, "1") == gnd))
                    })
                {
                    issues.push(ElectricalIssue {
                        component: f.reference.clone(),
                        message: format!(
                            "Buck-boost {name} requires its specified grounded {value} capacitor"
                        ),
                    });
                }
            }
            if let Some(mcu) = board
                .footprints
                .iter()
                .find(|m| m.value.starts_with("RP2040"))
            {
                let vbus = net("5V_VBUS");
                let detect = pin_net(mcu, "36");
                if !resistor_between(board, vbus, detect, Some("5.6k"))
                    || !resistor_between(board, detect, gnd, Some("10k"))
                {
                    issues.push(ElectricalIssue {component:mcu.reference.clone(),message:"GPIO24 VBUS detection requires the specified 5.6k/10k divider, not a lone series connection".into()});
                }
                let sense = pin_net(mcu, "41");
                let isolation = board.footprints.iter().find(|q| q.value == "DMG1012T-7");
                let sense_ok = isolation.is_some_and(|q| {
                    let divider = pin_net(q, "3");
                    let filtered = divider.is_some()
                        && gnd.is_some()
                        && board.footprints.iter().any(|c| {
                            c.reference.starts_with('C')
                                && c.value == "100nF"
                                && ((pin_net(c, "1") == divider && pin_net(c, "2") == gnd)
                                    || (pin_net(c, "2") == divider && pin_net(c, "1") == gnd))
                        });
                    vdd.is_some()
                        && sense.is_some()
                        && sense != divider
                        && pin_net(q, "1") == vdd
                        && pin_net(q, "2") == sense
                        && resistor_between(board, vsys, divider, Some("200k"))
                        && resistor_between(board, divider, gnd, Some("100k"))
                        && filtered
                });
                if !sense_ok {
                    issues.push(ElectricalIssue {component:mcu.reference.clone(),message:"GPIO29 VSYS sensing requires the 200k/100k filtered divider and NMOS gate=3V3/source=ADC/drain=divider orientation to prevent unpowered backfeed".into()});
                }
            }
        }
    }
    if board
        .footprints
        .iter()
        .any(|f| f.value == "ESP32-WROVER-E-N4R8" || f.value == "AiThinker_OV2640_FPC24")
    {
        check_camera_circuit(board, &mut issues);
    }
    issues
}

// Selected camera/reference circuit requirements, not a general schematic ERC.
fn check_camera_circuit(board: &PcbBoard, issues: &mut Vec<ElectricalIssue>) {
    let net = |name: &str| {
        board
            .net_names
            .iter()
            .find_map(|(id, n)| (n == name).then_some(*id))
    };
    for (reference, value) in [
        ("U1", "ESP32-WROVER-E-N4R8"),
        ("U4", "CP2102N-A02-GQFN28"),
        ("U3", "LP5907MFX-2.8/NOPB"),
        ("U5", "LP5907MFX-1.2/NOPB"),
    ] {
        if !board
            .find_footprint(reference)
            .is_some_and(|f| f.value == value)
        {
            issues.push(ElectricalIssue {
                component: reference.into(),
                message: format!("Camera circuit requires the selected {value}"),
            });
        }
    }
    for (reference, pin, expected) in [
        ("U1", "1", "GND"),
        ("U1", "15", "GND"),
        ("U1", "38", "GND"),
        ("U1", "2", "3V3_SYS"),
        ("U1", "3", "ESP_EN"),
        ("U1", "25", "ESP_IO0"),
        ("U1", "34", "UART_RX0"),
        ("U1", "35", "UART_TX0"),
        ("U2", "1", "GND"),
        ("U2", "3", "EXT_5V"),
        ("U2", "2", "3V3_SYS"),
        ("U2", "4", "3V3_SYS"),
        ("J1", "A4", "USB_VBUS"),
        ("J1", "B4", "USB_VBUS"),
        ("U4", "3", "GND"),
        ("U4", "29", "GND"),
        ("U4", "7", "EXT_5V"),
        ("U4", "6", "USB_3V3"),
        ("U4", "8", "VBUS_SENSE"),
        ("U4", "9", "BRIDGE_RESET_N"),
        ("U4", "4", "USB_DP"),
        ("U4", "5", "USB_DN"),
        ("U4", "26", "UART_RX0"),
        ("U4", "25", "UART_TX0"),
        ("U4", "28", "UART_DTR"),
        ("U4", "24", "UART_RTS"),
        ("Q1", "1", "DTR_BASE"),
        ("Q1", "2", "UART_RTS"),
        ("Q1", "3", "ESP_EN"),
        ("Q2", "1", "RTS_BASE"),
        ("Q2", "2", "UART_DTR"),
        ("Q2", "3", "ESP_IO0"),
        ("J2", "3", "CAM_D2"),
        ("J2", "4", "CAM_D1"),
        ("J2", "5", "CAM_D3"),
        ("J2", "6", "CAM_D0"),
        ("J2", "7", "CAM_D4"),
        ("J2", "8", "CAM_PCLK"),
        ("J2", "9", "CAM_D5"),
        ("J2", "10", "GND"),
        ("J2", "11", "CAM_D6"),
        ("J2", "12", "ESP_IO0"),
        ("J2", "13", "CAM_D7"),
        ("J2", "14", "3V3_SYS"),
        ("J2", "15", "CAM_DVDD"),
        ("J2", "16", "CAM_HREF"),
        ("J2", "17", "CAM_PWDN"),
        ("J2", "18", "CAM_VSYNC"),
        ("J2", "19", "CAM_RESET_N"),
        ("J2", "20", "CAM_SIOC"),
        ("J2", "21", "CAM_AVDD"),
        ("J2", "22", "CAM_SIOD"),
        ("J2", "23", "GND"),
        ("U1", "29", "CAM_D0"),
        ("U1", "30", "CAM_D1"),
        ("U1", "31", "CAM_D2"),
        ("U1", "33", "CAM_D3"),
        ("U1", "4", "CAM_D4"),
        ("U1", "5", "CAM_D5"),
        ("U1", "6", "CAM_D6"),
        ("U1", "7", "CAM_D7"),
        ("U1", "36", "CAM_PCLK"),
        ("U1", "37", "CAM_HREF"),
        ("U1", "8", "CAM_PWDN"),
        ("U1", "10", "CAM_VSYNC"),
        ("U1", "12", "CAM_SIOC"),
        ("U1", "11", "CAM_SIOD"),
        ("U1", "14", "SD_CLK"),
        ("J3", "5", "SD_CLK"),
        ("U1", "16", "SD_MISO"),
        ("J3", "7", "SD_MISO"),
        ("U1", "9", "SD_MOSI"),
        ("J3", "3", "SD_MOSI"),
        ("U1", "23", "SD_CS"),
        ("J3", "2", "SD_CS"),
        ("J3", "4", "3V3_SYS"),
        ("J3", "6", "GND"),
    ] {
        if net(expected).is_none()
            || board
                .find_footprint(reference)
                .and_then(|f| pin_net(f, pin))
                != net(expected)
        {
            issues.push(ElectricalIssue {
                component: reference.into(),
                message: format!("Camera circuit pin {pin} requires {expected}"),
            });
        }
    }
    for (reference, output) in [("U3", "CAM_AVDD"), ("U5", "CAM_DVDD")] {
        for (pin, rail) in [
            ("1", "3V3_SYS"),
            ("2", "GND"),
            ("3", "3V3_SYS"),
            ("5", output),
        ] {
            if net(rail).is_none()
                || board
                    .find_footprint(reference)
                    .and_then(|f| pin_net(f, pin))
                    != net(rail)
            {
                issues.push(ElectricalIssue {
                    component: reference.into(),
                    message: format!("Camera LDO pin {pin} requires {rail}"),
                });
            }
        }
    }
    for (a, b, value) in [
        ("UART_DTR", "DTR_BASE", "10k"),
        ("UART_RTS", "RTS_BASE", "10k"),
        ("3V3_SYS", "ESP_EN", "10k"),
        ("3V3_SYS", "ESP_IO0", "10k"),
        ("USB_3V3", "BRIDGE_RESET_N", "1k"),
        ("USB_VBUS", "VBUS_SENSE", "22.1k"),
        ("VBUS_SENSE", "GND", "47.5k"),
        ("3V3_SYS", "CAM_SIOC", "4.7k"),
        ("3V3_SYS", "CAM_SIOD", "4.7k"),
        ("3V3_SYS", "CAM_RESET_N", "4.7k"),
        ("SD_CLK", "GND", "10k"),
        ("3V3_SYS", "SD_MISO", "10k"),
        ("3V3_SYS", "SD_MOSI", "10k"),
        ("3V3_SYS", "SD_CS", "10k"),
    ] {
        if !resistor_between(board, net(a), net(b), Some(value)) {
            issues.push(ElectricalIssue {
                component: "ESP32 camera".into(),
                message: format!("Required {value} resistor between {a} and {b} is missing"),
            });
        }
    }
    for (rail, value) in [
        ("CAM_AVDD", "1uF"),
        ("CAM_DVDD", "1uF"),
        ("ESP_EN", "1uF"),
        ("USB_3V3", "4.7uF"),
    ] {
        if !board.footprints.iter().any(|c| {
            c.reference.starts_with('C')
                && c.value == value
                && pin_net(c, "1") == net(rail)
                && pin_net(c, "2") == net("GND")
        }) {
            issues.push(ElectricalIssue {
                component: "ESP32 camera".into(),
                message: format!("{rail} requires the specified grounded {value} capacitor"),
            });
        }
    }
}
