//! RP2040 electrical connections shared by the two development-board variants.
//! Pin numbers: Raspberry Pi RP2040 datasheet section 5.5. Power/clock/boot:
//! https://datasheets.raspberrypi.com/rp2040/hardware-design-with-rp2040.pdf
use super::*;

pub(crate) fn passive(reference: &str, value: &str, pos: Point, rotation: f64) -> Footprint {
    let mut f = resistor_0805(reference, value);
    f.package = if reference.starts_with('C') {
        "C_0402_1005Metric"
    } else {
        "R_0402_1005Metric"
    }
    .into();
    for (i, p) in f.pads.iter_mut().enumerate() {
        p.relative_pos = Point::new(if i == 0 { -0.5 } else { 0.5 }, 0.0);
        p.shape = PadShape::Rect {
            width: 0.6,
            height: 0.6,
        };
    }
    f.silkscreen_lines.clear();
    f.at_position(pos).with_rotation(rotation)
}

// Keep added bypass capacitors out of existing component envelopes. This is
// placement assistance only; the final power-loop layout still requires review.
pub(crate) fn add_passive(board: &mut PcbBoard, mut f: Footprint) {
    let clear = |f: &Footprint| {
        let b = f.world_bounding_box().inflate(0.2);
        b.min.x >= 0.4
            && b.min.y >= 0.4
            && b.max.x <= board.width - 0.4
            && b.max.y <= board.height - 0.4
            && board
                .footprints
                .iter()
                .all(|other| !b.intersects(&other.world_bounding_box()))
    };
    if !clear(&f) {
        let desired = f.position;
        let mut candidates = Vec::new();
        for x in 4..(board.width * 4.0) as i32 - 3 {
            for y in 4..(board.height * 4.0) as i32 - 3 {
                let p = Point::new(x as f64 / 4.0, y as f64 / 4.0);
                candidates.push(p);
            }
        }
        candidates.sort_by(|a, b| a.distance_to(desired).total_cmp(&b.distance_to(desired)));
        for pos in candidates {
            f.position = pos;
            if clear(&f) {
                break;
            }
        }
    }
    board.add_footprint(f);
}

pub fn complete_core(board: &mut PcbBoard) {
    let vdd = board.get_or_create_net("3V3");
    let core = board.get_or_create_net("1V1_CORE");
    let gnd = board.get_or_create_net("GND");
    let adc = board
        .net_names
        .iter()
        .find(|(_, name)| name.as_str() == "ADC_VREF")
        .map(|(net, _)| *net)
        .unwrap_or(vdd);
    for p in ["1", "10", "22", "33", "42", "44", "48", "49"] {
        board.connect("U1", p, vdd);
    }
    board.connect("U1", "43", adc);
    for p in ["23", "45", "50"] {
        board.connect("U1", p, core);
    }
    for p in ["19", "57"] {
        board.connect("U1", p, gnd);
    }
    for (name, mcu, flash) in [("QSPI_D2", "54", "3"), ("QSPI_D3", "51", "7")] {
        let net = board.get_or_create_net(name);
        board.connect("U1", mcu, net);
        board.connect("U2", flash, net);
    }
    // Every duplicated contact must be connected, including actual GND contacts.
    for p in ["A1", "B1", "SH1", "SH2", "SH3", "SH4"] {
        board.connect("J1", p, gnd);
    }
    let j = board.find_footprint("J1").unwrap();
    let power = j
        .pads
        .iter()
        .find(|p| p.pin_number == "A4")
        .unwrap()
        .net
        .unwrap();
    let dp = j
        .pads
        .iter()
        .find(|p| p.pin_number == "A6" || p.pin_number == "B6")
        .and_then(|p| p.net)
        .or_else(|| {
            j.pads
                .iter()
                .find(|p| p.pin_number == "B6")
                .and_then(|p| p.net)
        })
        .unwrap();
    let dm = j
        .pads
        .iter()
        .find(|p| p.pin_number == "A7")
        .unwrap()
        .net
        .unwrap();
    board.connect("J1", "B4", power);
    for p in ["A6", "B6"] {
        board.connect("J1", p, dp);
    }
    for p in ["A7", "B7"] {
        board.connect("J1", p, dm);
    }
    let center = board.find_footprint("U1").unwrap().position;
    // One bypass per IOVDD group, USB and ADC supply; two core bypasses.
    for (i, (x, y, rot, net)) in [
        (-6.0, -2.6, 90.0, vdd),
        (-6.0, 1.0, 90.0, vdd),
        (0.2, 6.0, 0.0, vdd),
        (6.0, 1.0, 90.0, vdd),
        (6.0, -2.6, 90.0, vdd),
        (0.2, -6.0, 0.0, vdd),
        (2.3, -6.0, 0.0, vdd),
        (-2.0, -6.0, 0.0, adc),
        (-2.0, 6.0, 0.0, core),
        (0.2, -8.0, 0.0, core),
    ]
    .into_iter()
    .enumerate()
    {
        let reference = format!("C{}", 20 + i);
        add_passive(
            board,
            passive(&reference, "100nF", center + Point::new(x, y), rot),
        );
        board.connect(&reference, "1", net);
        board.connect(&reference, "2", gnd);
    }
    // VREG_VIN local 1uF and flash supply bypass.
    for (r, pos, value) in [
        ("C30", center + Point::new(2.3, -8.0), "1uF"),
        (
            "C31",
            board.find_footprint("U2").unwrap().position + Point::new(0.0, -3.5),
            "100nF",
        ),
    ] {
        add_passive(board, passive(r, value, pos, 0.0));
        board.connect(r, "1", vdd);
        board.connect(r, "2", gnd);
    }
}
