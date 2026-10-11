//! Parametric component footprint library.
//!
//! Provides IPC-7351B compliant footprint generators for standard SMD and THT packages:
//! - TQFP-32 (ATmega328P microcontroller)
//! - SOT-223 / SOT-23 (3-pin & tab voltage regulators)
//! - USB-C 16-pin receptacle (hybrid SMD + THT shell retention tabs)
//! - 0805 passives (resistors, capacitors, polarized LEDs)
//! - Crystal resonators (HC-49/US SMD and SMD 3225 4-pin)
//! - 1x15 0.1" (2.54 mm) through-hole expansion headers
//! - 3x4 mm tactile pushbutton switch (RESET button)

use crate::pcb::core::{Footprint, Layer, Pad, PadShape, Point};

// ============================================================================
// 1. TQFP-32 Footprint (ATmega328P Microcontroller)
// ============================================================================

/// Generates a 32-pin Quad Flat Pack footprint matching IPC-7351B standards.
///
/// - Body: 7.0 x 7.0 mm
/// - Pitch: 0.80 mm
/// - Pad size: 0.50 x 1.50 mm (width x length)
/// - Pad center radius: 4.25 mm
pub fn tqfp32(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pin_names: [&str; 32] = [
        "PD3", "PD4", "GND", "VCC", "GND", "VCC", "PB6", "PB7", // Pins 1-8 (Left)
        "PD5", "PD6", "PD7", "PB0", "PB1", "PB2", "PB3", "PB4", // Pins 9-16 (Bottom)
        "PB5", "AVCC", "ADC6", "AREF", "GND", "ADC7", "PC0", "PC1", // Pins 17-24 (Right)
        "PC2", "PC3", "PC4", "PC5", "PC6", "PD0", "PD1", "PD2", // Pins 25-32 (Top)
    ];
    tqfp32_with_pin_names(reference, value, &pin_names)
}

/// Generates a TQFP-32 footprint with custom pin names.
pub fn tqfp32_with_pin_names(
    reference: impl Into<String>,
    value: impl Into<String>,
    pin_names: &[&str; 32],
) -> Footprint {
    let pitch = 0.80;
    let pad_w = 0.50;
    let pad_l = 1.50;
    let radius = 4.25;

    let mut pads = Vec::with_capacity(32);

    let offsets: [f64; 8] = [
        -3.5 * pitch,
        -2.5 * pitch,
        -1.5 * pitch,
        -0.5 * pitch,
        0.5 * pitch,
        1.5 * pitch,
        2.5 * pitch,
        3.5 * pitch,
    ];

    // Side 1: Left (Pins 1..8, Y goes from top to bottom)
    for i in 0..8 {
        let pin_num = (i + 1).to_string();
        let rel_pos = Point::new(-radius, offsets[i]);
        let shape = PadShape::Rect {
            width: pad_l,
            height: pad_w,
        };
        pads.push(Pad::new_smd(
            pin_num,
            pin_names[i],
            rel_pos,
            shape,
            Layer::FCu,
        ));
    }

    // Side 2: Bottom (Pins 9..16, X goes from left to right)
    for i in 0..8 {
        let pin_num = (i + 9).to_string();
        let rel_pos = Point::new(offsets[i], radius);
        let shape = PadShape::Rect {
            width: pad_w,
            height: pad_l,
        };
        pads.push(Pad::new_smd(
            pin_num,
            pin_names[i + 8],
            rel_pos,
            shape,
            Layer::FCu,
        ));
    }

    // Side 3: Right (Pins 17..24, Y goes from bottom to top)
    for i in 0..8 {
        let pin_num = (i + 17).to_string();
        let rel_pos = Point::new(radius, offsets[7 - i]);
        let shape = PadShape::Rect {
            width: pad_l,
            height: pad_w,
        };
        pads.push(Pad::new_smd(
            pin_num,
            pin_names[i + 16],
            rel_pos,
            shape,
            Layer::FCu,
        ));
    }

    // Side 4: Top (Pins 25..32, X goes from right to left)
    for i in 0..8 {
        let pin_num = (i + 25).to_string();
        let rel_pos = Point::new(offsets[7 - i], -radius);
        let shape = PadShape::Rect {
            width: pad_w,
            height: pad_l,
        };
        pads.push(Pad::new_smd(
            pin_num,
            pin_names[i + 24],
            rel_pos,
            shape,
            Layer::FCu,
        ));
    }

    // Silkscreen: 7.0 x 7.0 mm body with beveled top-left corner (Pin 1 indicator)
    let body_half = 3.50;
    let bevel = 1.0;
    let silkscreen_lines = vec![
        (
            Point::new(-body_half + bevel, -body_half),
            Point::new(-body_half, -body_half + bevel),
        ),
        (
            Point::new(-body_half, -body_half + bevel),
            Point::new(-body_half, body_half),
        ),
        (
            Point::new(-body_half, body_half),
            Point::new(body_half, body_half),
        ),
        (
            Point::new(body_half, body_half),
            Point::new(body_half, -body_half),
        ),
        (
            Point::new(body_half, -body_half),
            Point::new(-body_half + bevel, -body_half),
        ),
        (
            Point::new(-body_half - 0.5, -body_half),
            Point::new(-body_half - 1.2, -body_half),
        ),
    ];

    Footprint::new(
        reference,
        value,
        "TQFP-32_7x7mm_P0.8mm",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 2. SOT-223 / SOT-23 Footprints (Voltage Regulators)
// ============================================================================

/// Generates a SOT-223 3-pin + Tab regulator footprint (e.g. AMS1117-3.3).
///
/// - Pin 1: GND / ADJ (X = -2.30, Y = +3.10)
/// - Pin 2: VOUT (X = 0.00, Y = +3.10)
/// - Pin 3: VIN (X = +2.30, Y = +3.10)
/// - Pin 4: VOUT / TAB (X = 0.00, Y = -3.10, 3.3 x 1.8 mm)
pub fn sot223(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pad_w = 1.00;
    let pad_h = 1.80;
    let sig_y = 3.10;
    let tab_y = -3.10;

    let pads = vec![
        Pad::new_smd(
            "1",
            "GND/ADJ",
            Point::new(-2.30, sig_y),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "2",
            "VOUT",
            Point::new(0.00, sig_y),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "3",
            "VIN",
            Point::new(2.30, sig_y),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "4",
            "TAB",
            Point::new(0.00, tab_y),
            PadShape::Rect {
                width: 3.30,
                height: pad_h,
            },
            Layer::FCu,
        ),
    ];

    let silkscreen_lines = vec![
        (Point::new(-3.25, -1.75), Point::new(3.25, -1.75)),
        (Point::new(3.25, -1.75), Point::new(3.25, 1.75)),
        (Point::new(3.25, 1.75), Point::new(-3.25, 1.75)),
        (Point::new(-3.25, 1.75), Point::new(-3.25, -1.75)),
        (Point::new(-3.25, 1.25), Point::new(-3.75, 1.75)),
    ];

    Footprint::new(
        reference,
        value,
        "SOT-223-3_TabPin2",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

/// Generates a SOT-23 3-pin small outline transistor / regulator footprint.
pub fn sot23(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pad_w = 0.60;
    let pad_h = 1.00;
    let sig_y = 1.10;

    let pads = vec![
        Pad::new_smd(
            "1",
            "GND",
            Point::new(-0.95, sig_y),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "2",
            "VOUT",
            Point::new(0.95, sig_y),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "3",
            "VIN",
            Point::new(0.00, -sig_y),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
    ];

    let silkscreen_lines = vec![
        (Point::new(-1.45, -0.65), Point::new(1.45, -0.65)),
        (Point::new(1.45, -0.65), Point::new(1.45, 0.65)),
        (Point::new(1.45, 0.65), Point::new(-1.45, 0.65)),
        (Point::new(-1.45, 0.65), Point::new(-1.45, -0.65)),
    ];

    Footprint::new(
        reference,
        value,
        "SOT-23",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 3. USB-C 16-Pin Receptacle
// ============================================================================

/// Generates a standard 16-pin USB Type-C receptacle footprint.
///
/// Features:
/// - 12 SMD signal/power contacts at 0.50 mm pitch (0.30 x 1.00 mm).
/// - 4 Through-hole shell retention tabs (drill 1.0 mm, outer pad 1.8 mm).
pub fn usb_c_16pin(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let mut pads = Vec::with_capacity(16);

    let smd_specs = [
        ("A1", "GND", -2.75),
        ("A4", "VBUS", -2.25),
        ("A5", "CC1", -1.75),
        ("A6", "DP1", -1.25),
        ("A7", "DN1", -0.75),
        ("A8", "SBU1", -0.25),
        ("B8", "SBU2", 0.25),
        ("B7", "DN2", 0.75),
        ("B6", "DP2", 1.25),
        ("B5", "CC2", 1.75),
        ("B4", "VBUS", 2.25),
        ("B1", "GND", 2.75),
    ];

    for (num, name, x) in smd_specs {
        pads.push(Pad::new_smd(
            num,
            name,
            Point::new(x, 0.0),
            PadShape::Rect {
                width: 0.30,
                height: 1.00,
            },
            Layer::FCu,
        ));
    }

    let tht_tabs = [
        ("SH1", "GND", Point::new(-4.32, -1.50)),
        ("SH2", "GND", Point::new(4.32, -1.50)),
        ("SH3", "GND", Point::new(-4.32, 3.50)),
        ("SH4", "GND", Point::new(4.32, 3.50)),
    ];

    for (num, name, pos) in tht_tabs {
        pads.push(Pad::new_tht(
            num,
            name,
            pos,
            PadShape::Circle { diameter: 1.80 },
            1.00,
            1.80,
        ));
    }

    let silkscreen_lines = vec![
        (Point::new(-4.47, -2.50), Point::new(4.47, -2.50)),
        (Point::new(4.47, -2.50), Point::new(4.47, 4.85)),
        (Point::new(4.47, 4.85), Point::new(-4.47, 4.85)),
        (Point::new(-4.47, 4.85), Point::new(-4.47, -2.50)),
        (Point::new(-4.00, -2.50), Point::new(4.00, -2.50)),
    ];

    Footprint::new(
        reference,
        value,
        "USB_C_Receptacle_16Pin",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 4. 0805 Passives (Resistors, Capacitors, LEDs)
// ============================================================================

/// Generates an IPC-7351B 0805 metric 2012 surface-mount resistor footprint.
pub fn resistor_0805(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    passive_0805_internal(reference, value, "R_0805_2012Metric", false)
}

/// Generates an IPC-7351B 0805 metric 2012 surface-mount capacitor footprint.
pub fn capacitor_0805(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    passive_0805_internal(reference, value, "C_0805_2012Metric", false)
}

/// Generates an IPC-7351B 0805 surface-mount LED footprint with cathode polarity indicator.
pub fn led_0805(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    passive_0805_internal(reference, value, "LED_0805_2012Metric", true)
}

fn passive_0805_internal(
    reference: impl Into<String>,
    value: impl Into<String>,
    package: &'static str,
    is_led: bool,
) -> Footprint {
    let pad_w = 1.00;
    let pad_h = 1.30;
    let pitch_half = 0.95; // 1.90 mm center-to-center pitch

    let pin1_name = if is_led { "K" } else { "1" };
    let pin2_name = if is_led { "A" } else { "2" };

    let pads = vec![
        Pad::new_smd(
            "1",
            pin1_name,
            Point::new(-pitch_half, 0.0),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "2",
            pin2_name,
            Point::new(pitch_half, 0.0),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
    ];

    let mut silkscreen_lines = vec![
        (Point::new(-0.50, -0.85), Point::new(0.50, -0.85)),
        (Point::new(-0.50, 0.85), Point::new(0.50, 0.85)),
    ];

    if is_led {
        silkscreen_lines.push((Point::new(-1.65, -0.75), Point::new(-1.65, 0.75)));
    }

    Footprint::new(
        reference,
        value,
        package,
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 5. Crystal Resonator Footprints
// ============================================================================

/// Generates an HC-49/US SMD quartz crystal resonator footprint (2 signal pads + ground shield).
pub fn crystal_hc49_smd(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pads = vec![
        Pad::new_smd(
            "1",
            "XTAL1",
            Point::new(-4.75, 0.0),
            PadShape::Rect {
                width: 2.00,
                height: 1.50,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "2",
            "XTAL2",
            Point::new(4.75, 0.0),
            PadShape::Rect {
                width: 2.00,
                height: 1.50,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "3",
            "GND",
            Point::new(0.0, 0.0),
            PadShape::Rect {
                width: 3.00,
                height: 2.00,
            },
            Layer::FCu,
        ),
    ];

    let silkscreen_lines = vec![
        (Point::new(-4.00, -2.35), Point::new(4.00, -2.35)),
        (Point::new(4.00, -2.35), Point::new(5.70, 0.00)),
        (Point::new(5.70, 0.00), Point::new(4.00, 2.35)),
        (Point::new(4.00, 2.35), Point::new(-4.00, 2.35)),
        (Point::new(-4.00, 2.35), Point::new(-5.70, 0.00)),
        (Point::new(-5.70, 0.00), Point::new(-4.00, -2.35)),
    ];

    Footprint::new(
        reference,
        value,
        "Crystal_HC49_SMD",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

/// Generates an SMD 3225 4-pin ceramic crystal resonator footprint.
pub fn crystal_smd_3225(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pad_w = 1.20;
    let pad_h = 1.00;

    let pads = vec![
        Pad::new_smd(
            "1",
            "XTAL1",
            Point::new(-1.00, 0.85),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "2",
            "GND",
            Point::new(1.00, 0.85),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "3",
            "XTAL2",
            Point::new(1.00, -0.85),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "4",
            "GND",
            Point::new(-1.00, -0.85),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
    ];

    let silkscreen_lines = vec![
        (Point::new(-1.60, -1.25), Point::new(1.60, -1.25)),
        (Point::new(1.60, -1.25), Point::new(1.60, 1.25)),
        (Point::new(1.60, 1.25), Point::new(-1.10, 1.25)),
        (Point::new(-1.10, 1.25), Point::new(-1.60, 0.75)),
        (Point::new(-1.60, 0.75), Point::new(-1.60, -1.25)),
    ];

    Footprint::new(
        reference,
        value,
        "Crystal_SMD_3225-4Pin",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 6. Dual 1x15 0.1" (2.54 mm) Through-Hole Expansion Headers
// ============================================================================

/// Generates a single 1x15 0.1" (2.54 mm) through-hole pin header footprint.
///
/// - Pitch: 2.54 mm
/// - Drill: 1.00 mm
/// - Pad diameter: 1.70 mm
/// - Pin 1: Square pad (polarity indicator)
/// - Pins 2..15: Circular pads
pub fn header_1x15(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pin_names: Vec<String> = (1..=15).map(|n| n.to_string()).collect();
    let name_refs: Vec<&str> = pin_names.iter().map(|s| s.as_str()).collect();
    header_1x15_with_pin_names(reference, value, &name_refs)
}

/// Generates a 1x15 through-hole pin header footprint with specified pin names.
pub fn header_1x15_with_pin_names(
    reference: impl Into<String>,
    value: impl Into<String>,
    pin_names: &[&str],
) -> Footprint {
    let pitch = 2.54;
    let drill = 1.00;
    let pad_dia = 1.70;

    let mut pads = Vec::with_capacity(15);

    for i in 0..15 {
        let pin_num = (i + 1).to_string();
        let name = match pin_names.get(i) {
            Some(&n) => n.to_string(),
            None => pin_num.clone(),
        };
        let y_pos = (i as f64) * pitch;
        let pos = Point::new(0.0, y_pos);

        let shape = if i == 0 {
            PadShape::Rect {
                width: pad_dia,
                height: pad_dia,
            }
        } else {
            PadShape::Circle { diameter: pad_dia }
        };

        pads.push(Pad::new_tht(pin_num, name, pos, shape, drill, pad_dia));
    }

    let half_w = 1.27;
    let y_min = -1.27;
    let y_max = 14.0 * pitch + 1.27;
    let silkscreen_lines = vec![
        (Point::new(-half_w, y_min), Point::new(half_w, y_min)),
        (Point::new(half_w, y_min), Point::new(half_w, y_max)),
        (Point::new(half_w, y_max), Point::new(-half_w, y_max)),
        (Point::new(-half_w, y_max), Point::new(-half_w, y_min)),
    ];

    Footprint::new(
        reference,
        value,
        "PinHeader_1x15_P2.54mm_Vertical",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

/// Generates a pair of 1x15 breakout headers with dual-row breadboard spacing.
pub fn dual_headers_1x15(
    ref_left: impl Into<String>,
    ref_right: impl Into<String>,
    spacing_x: f64,
    left_pin_names: &[&str],
    right_pin_names: &[&str],
) -> (Footprint, Footprint) {
    let half_x = spacing_x / 2.0;
    let header_left = header_1x15_with_pin_names(ref_left, "Breakout_Left", left_pin_names)
        .at_position(Point::new(-half_x, 0.0));
    let header_right = header_1x15_with_pin_names(ref_right, "Breakout_Right", right_pin_names)
        .at_position(Point::new(half_x, 0.0));

    (header_left, header_right)
}

// ============================================================================
// 7. Tactile Pushbutton Switch (RESET Button)
// ============================================================================

/// Generates a 3x4 mm SMD tactile pushbutton switch footprint (4 terminals).
pub fn tactile_switch_3x4(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pad_w = 1.00;
    let pad_h = 1.00;
    let dx = 1.80;
    let dy = 1.00;

    let pads = vec![
        Pad::new_smd(
            "1",
            "1",
            Point::new(-dx, -dy),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "2",
            "2",
            Point::new(dx, -dy),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "3",
            "3",
            Point::new(-dx, dy),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
        Pad::new_smd(
            "4",
            "4",
            Point::new(dx, dy),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ),
    ];

    let silkscreen_lines = vec![
        (Point::new(-1.50, -2.00), Point::new(1.50, -2.00)),
        (Point::new(1.50, -2.00), Point::new(1.50, 2.00)),
        (Point::new(1.50, 2.00), Point::new(-1.50, 2.00)),
        (Point::new(-1.50, 2.00), Point::new(-1.50, -2.00)),
    ];

    Footprint::new(
        reference,
        value,
        "SW_Push_SPST_NO_3x4mm",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 8. SOIC-8 Footprint (CH340N USB-UART, W25Q128 Flash)
// ============================================================================

/// Generates a standard SOIC-8 (SOP-8) footprint (e.g. CH340N USB-UART bridge, W25Q128 Flash).
///
/// - Pitch: 1.27 mm
/// - Pad size: 1.50 x 0.60 mm
/// - Pad radius: 2.70 mm
pub fn soic8(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pad_w = 1.50;
    let pad_h = 0.60;
    let radius = 2.70;
    let pitch = 1.27;

    let pin_names = ["1", "2", "3", "4", "5", "6", "7", "8"];
    let mut pads = Vec::with_capacity(8);

    // Left side (Pins 1..4, top to bottom)
    for i in 0..4 {
        let y = -1.5 * pitch + (i as f64) * pitch;
        pads.push(Pad::new_smd(
            (i + 1).to_string(),
            pin_names[i],
            Point::new(-radius, y),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ));
    }

    // Right side (Pins 5..8, bottom to top)
    for i in 0..4 {
        let y = 1.5 * pitch - (i as f64) * pitch;
        pads.push(Pad::new_smd(
            (i + 5).to_string(),
            pin_names[i + 4],
            Point::new(radius, y),
            PadShape::Rect {
                width: pad_w,
                height: pad_h,
            },
            Layer::FCu,
        ));
    }

    let silkscreen_lines = vec![
        (Point::new(-2.0, -2.5), Point::new(2.0, -2.5)),
        (Point::new(2.0, -2.5), Point::new(2.0, 2.5)),
        (Point::new(2.0, 2.5), Point::new(-2.0, 2.5)),
        (Point::new(-2.0, 2.5), Point::new(-2.0, -2.5)),
        (Point::new(-2.0, -1.8), Point::new(-2.8, -1.8)),
    ];

    Footprint::new(
        reference,
        value,
        "SOIC-8_3.9x4.9mm_P1.27mm",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 9. SOT-23-5 Footprint (MCP73831 Charger, LP5907 Camera LDO)
// ============================================================================

/// Generates a standard SOT-23-5 footprint (e.g. MCP73831 LiPo charger, LP5907 camera LDO).
pub fn sot23_5(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pad_w = 1.00;
    let pad_h = 0.50;
    let x_off = 1.30;
    let pitch = 0.95;

    let pads = vec![
        Pad::new_smd("1", "1", Point::new(-x_off, -pitch), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
        Pad::new_smd("2", "2", Point::new(-x_off, 0.0), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
        Pad::new_smd("3", "3", Point::new(-x_off, pitch), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
        Pad::new_smd("4", "4", Point::new(x_off, pitch), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
        Pad::new_smd("5", "5", Point::new(x_off, -pitch), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
    ];

    let silkscreen_lines = vec![
        (Point::new(-0.85, -1.5), Point::new(0.85, -1.5)),
        (Point::new(0.85, -1.5), Point::new(0.85, 1.5)),
        (Point::new(0.85, 1.5), Point::new(-0.85, 1.5)),
        (Point::new(-0.85, 1.5), Point::new(-0.85, -1.5)),
        (Point::new(-0.85, -1.0), Point::new(-1.8, -1.0)),
    ];

    Footprint::new(
        reference,
        value,
        "SOT-23-5",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// Exact AP2112 SOT25 reference lands: Diodes DS39724 Rev. 2-2, page 14.
// The generic SOT-23-5 template remains available for other parts.
pub fn ap2112k_33(reference: impl Into<String>) -> Footprint {
    let mut footprint = sot23_5(reference, "AP2112K-3.3TRG1");
    for pad in &mut footprint.pads {
        pad.relative_pos.x = pad.relative_pos.x.signum() * 1.20;
        pad.shape = PadShape::Rect { width: 0.80, height: 0.55 };
    }
    footprint.silkscreen_lines = vec![
        (Point::new(-0.65, -1.5), Point::new(0.65, -1.5)),
        (Point::new(0.65, 1.5), Point::new(-0.65, 1.5)),
    ];
    footprint
}

// ============================================================================
// 10. FPC 24-Pin 0.5mm Connector Footprint (OV2640 / OV5640 Camera)
// ============================================================================

/// Generates a 24-pin 0.5 mm pitch horizontal FPC camera connector.
pub fn fpc_24pin_05mm(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let mut pads = Vec::with_capacity(26);
    let pitch = 0.50;
    let pad_w = 0.30;
    let pad_h = 1.00;

    let pin_names = [
        "STROBE", "AGND", "SIOD", "AVDD", "SIOC", "RESET",
        "VSYNC", "PWDN", "HREF", "DVDD", "DOVDD", "Y9",
        "XCLK", "Y8", "DGND", "Y7", "PCLK", "Y6",
        "Y2", "Y5", "Y3", "Y4", "GND1", "GND2",
    ];

    for i in 0..24 {
        let x = -5.75 + (i as f64) * pitch;
        pads.push(Pad::new_smd(
            (i + 1).to_string(),
            pin_names[i],
            Point::new(x, 0.0),
            PadShape::Rect { width: pad_w, height: pad_h },
            Layer::FCu,
        ));
    }

    pads.push(Pad::new_smd("MP1", "GND", Point::new(-6.80, -1.20), PadShape::Rect { width: 1.20, height: 1.60 }, Layer::FCu));
    pads.push(Pad::new_smd("MP2", "GND", Point::new(6.80, -1.20), PadShape::Rect { width: 1.20, height: 1.60 }, Layer::FCu));

    let silkscreen_lines = vec![
        (Point::new(-7.2, -2.5), Point::new(7.2, -2.5)),
        (Point::new(7.2, -2.5), Point::new(7.2, 1.2)),
        (Point::new(7.2, 1.2), Point::new(-7.2, 1.2)),
        (Point::new(-7.2, 1.2), Point::new(-7.2, -2.5)),
        (Point::new(-6.0, -2.5), Point::new(-6.0, -3.2)),
    ];

    Footprint::new(
        reference,
        value,
        "FPC_24Pin_P0.5mm_Horizontal",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 11. MicroSD Socket Footprint
// ============================================================================

/// Generates a standard surface-mount MicroSD card socket footprint.
pub fn microsd_socket(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let mut pads = Vec::with_capacity(12);

    let sig_pads = [
        ("1", "DAT2", -3.50, -2.50),
        ("2", "CD/DAT3", -2.40, -2.50),
        ("3", "CMD", -1.30, -2.50),
        ("4", "VDD", -0.20, -2.50),
        ("5", "CLK", 0.90, -2.50),
        ("6", "VSS", 2.00, -2.50),
        ("7", "DAT0", 3.10, -2.50),
        ("8", "DAT1", 4.20, -2.50),
    ];

    for (num, name, x, y) in sig_pads {
        pads.push(Pad::new_smd(
            num,
            name,
            Point::new(x, y),
            PadShape::Rect { width: 0.70, height: 1.40 },
            Layer::FCu,
        ));
    }

    let shield_tabs = [
        ("SH1", "GND", Point::new(-6.80, -4.50)),
        ("SH2", "GND", Point::new(6.80, -4.50)),
        ("SH3", "GND", Point::new(-6.80, 5.00)),
        ("SH4", "GND", Point::new(6.80, 5.00)),
    ];

    for (num, name, pos) in shield_tabs {
        pads.push(Pad::new_smd(
            num,
            name,
            pos,
            PadShape::Rect { width: 1.40, height: 1.60 },
            Layer::FCu,
        ));
    }

    let silkscreen_lines = vec![
        (Point::new(-7.0, -5.5), Point::new(7.0, -5.5)),
        (Point::new(7.0, -5.5), Point::new(7.0, 6.0)),
        (Point::new(7.0, 6.0), Point::new(-7.0, 6.0)),
        (Point::new(-7.0, 6.0), Point::new(-7.0, -5.5)),
    ];

    Footprint::new(
        reference,
        value,
        "MicroSD_Socket_PushPush",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 12. ESP32-WROOM-32 / ESP32-S Module Footprint
// ============================================================================

/// Generates an ESP32 module footprint (38 castellated pins).
pub fn esp32_wroom32(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let mut pads = Vec::with_capacity(38);
    let pitch = 1.27;
    let pad_w = 1.50;
    let pad_h = 0.65;
    let radius = 9.0;

    let left_pins = [
        "GND", "3V3", "EN", "SENSOR_VP", "SENSOR_VN", "IO34", "IO35",
        "IO32", "IO33", "IO25", "IO26", "IO27", "IO14", "IO12",
        "GND", "IO13", "SD2", "SD3", "CMD",
    ];

    let right_pins = [
        "CLK", "SD0", "SD1", "IO15", "IO2", "IO0", "IO4",
        "IO16", "IO17", "IO5", "IO18", "IO19", "GND", "IO21",
        "RXD0", "TXD0", "IO22", "IO23", "GND",
    ];

    for i in 0..19 {
        let y = -9.0 * pitch + (i as f64) * pitch;
        pads.push(Pad::new_smd(
            (i + 1).to_string(),
            left_pins[i],
            Point::new(-radius, y),
            PadShape::Rect { width: pad_w, height: pad_h },
            Layer::FCu,
        ));
    }

    for i in 0..19 {
        let y = 9.0 * pitch - (i as f64) * pitch;
        pads.push(Pad::new_smd(
            (i + 20).to_string(),
            right_pins[i],
            Point::new(radius, y),
            PadShape::Rect { width: pad_w, height: pad_h },
            Layer::FCu,
        ));
    }

    let silkscreen_lines = vec![
        (Point::new(-9.0, -12.75), Point::new(9.0, -12.75)),
        (Point::new(9.0, -12.75), Point::new(9.0, 12.75)),
        (Point::new(9.0, 12.75), Point::new(-9.0, 12.75)),
        (Point::new(-9.0, 12.75), Point::new(-9.0, -12.75)),
        (Point::new(-9.0, -8.0), Point::new(9.0, -8.0)),
    ];

    Footprint::new(
        reference,
        value,
        "ESP32-WROOM-32",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 13. QFN-56 Footprint (RP2040 Microcontroller)
// ============================================================================

/// Generates a QFN-56 7x7 mm 0.4 mm pitch footprint for the RP2040 microcontroller.
pub fn qfn56(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let mut pads = Vec::with_capacity(57);
    let pitch = 0.40;
    let pad_len = 0.70;
    let pad_w = 0.20;
    let radius = 3.45;

    let offsets: [f64; 14] = [
        -6.5 * pitch, -5.5 * pitch, -4.5 * pitch, -3.5 * pitch,
        -2.5 * pitch, -1.5 * pitch, -0.5 * pitch, 0.5 * pitch,
        1.5 * pitch, 2.5 * pitch, 3.5 * pitch, 4.5 * pitch,
        5.5 * pitch, 6.5 * pitch,
    ];

    // Raspberry Pi RP2040 datasheet, section 5.5, QFN-56 (not GPIO numbers).
    let pin_names = [
        "IOVDD1", "GPIO0", "GPIO1", "GPIO2", "GPIO3", "GPIO4", "GPIO5",
        "GPIO6", "GPIO7", "IOVDD2", "GPIO8", "GPIO9", "GPIO10", "GPIO11",
        "GPIO12", "GPIO13", "GPIO14", "GPIO15", "TESTEN", "XIN", "XOUT",
        "IOVDD3", "DVDD1", "SWCLK", "SWD", "RUN", "GPIO16", "GPIO17",
        "GPIO18", "GPIO19", "GPIO20", "GPIO21", "IOVDD4", "GPIO22", "GPIO23",
        "GPIO24", "GPIO25", "GPIO26", "GPIO27", "GPIO28", "GPIO29", "IOVDD5",
        "ADC_AVDD", "VREG_VIN", "VREG_VOUT", "USB_DM", "USB_DP", "USB_VDD",
        "IOVDD6", "DVDD2", "QSPI_SD3", "QSPI_SCLK", "QSPI_SD0", "QSPI_SD2",
        "QSPI_SD1", "QSPI_SS",
    ];

    // Left (Pins 1..14)
    for i in 0..14 {
        pads.push(Pad::new_smd(
            (i + 1).to_string(),
            pin_names[i],
            Point::new(-radius, offsets[i]),
            PadShape::Rect { width: pad_len, height: pad_w },
            Layer::FCu,
        ));
    }

    // Bottom (Pins 15..28)
    for i in 0..14 {
        pads.push(Pad::new_smd(
            (i + 15).to_string(),
            pin_names[i + 14],
            Point::new(offsets[i], radius),
            PadShape::Rect { width: pad_w, height: pad_len },
            Layer::FCu,
        ));
    }

    // Right (Pins 29..42)
    for i in 0..14 {
        pads.push(Pad::new_smd(
            (i + 29).to_string(),
            pin_names[i + 28],
            Point::new(radius, offsets[13 - i]),
            PadShape::Rect { width: pad_len, height: pad_w },
            Layer::FCu,
        ));
    }

    // Top (Pins 43..56)
    for i in 0..14 {
        pads.push(Pad::new_smd(
            (i + 43).to_string(),
            pin_names[i + 42],
            Point::new(offsets[13 - i], -radius),
            PadShape::Rect { width: pad_w, height: pad_len },
            Layer::FCu,
        ));
    }

    // Pin 57: Exposed ground pad (EPAD)
    pads.push(Pad::new_smd(
        "57",
        "GND_EPAD",
        Point::zero(),
        PadShape::Rect { width: 3.20, height: 3.20 },
        Layer::FCu,
    ));

    let silkscreen_lines = vec![
        (Point::new(-3.5, -3.5), Point::new(3.5, -3.5)),
        (Point::new(3.5, -3.5), Point::new(3.5, 3.5)),
        (Point::new(3.5, 3.5), Point::new(-3.5, 3.5)),
        (Point::new(-3.5, 3.5), Point::new(-3.5, -3.5)),
        (Point::new(-3.5, -2.5), Point::new(-4.2, -3.5)),
    ];

    Footprint::new(
        reference,
        value,
        "QFN-56-1EP_7x7mm_P0.4mm",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 14. JST-PH 2-Pin Connector Footprint (LiPo Battery)
// ============================================================================

/// Generates a JST-PH 2.0 mm pitch 2-pin LiPo battery connector.
pub fn jst_ph_2pin(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let drill = 0.80;
    let pad_dia = 1.60;

    let pads = vec![
        Pad::new_tht("1", "VBAT+", Point::new(-1.00, 0.0), PadShape::Rect { width: pad_dia, height: pad_dia }, drill, pad_dia),
        Pad::new_tht("2", "GND", Point::new(1.00, 0.0), PadShape::Circle { diameter: pad_dia }, drill, pad_dia),
    ];

    let silkscreen_lines = vec![
        (Point::new(-3.0, -2.25), Point::new(3.0, -2.25)),
        (Point::new(3.0, -2.25), Point::new(3.0, 2.25)),
        (Point::new(3.0, 2.25), Point::new(-3.0, 2.25)),
        (Point::new(-3.0, 2.25), Point::new(-3.0, -2.25)),
        (Point::new(-1.0, -2.7), Point::new(-1.0, -3.3)),
        (Point::new(-1.3, -3.0), Point::new(-0.7, -3.0)),
    ];

    Footprint::new(
        reference,
        value,
        "JST_PH_S2B-PH-K-S_1x02_P2.00mm_Vertical",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 15. JST-SH 4-Pin Connector Footprint (Qwiic / STEMMA QT)
// ============================================================================

/// JST SM04B-SRSS-TB lands from the official SH mounting-surface drawing.
/// Pin1-left signal-row origin; mounting row is +3.875 mm, with a 6x4.25 mm body.
pub fn jst_sh_4pin(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pad_w = 0.60;
    let pad_h = 1.55;

    let pads = vec![
        Pad::new_smd("1", "GND", Point::new(-1.50, 0.0), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
        Pad::new_smd("2", "3V3", Point::new(-0.50, 0.0), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
        Pad::new_smd("3", "SDA", Point::new(0.50, 0.0), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
        Pad::new_smd("4", "SCL", Point::new(1.50, 0.0), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
        Pad::new_smd("MP1", "GND", Point::new(-2.80, 3.875), PadShape::Rect { width: 1.20, height: 1.80 }, Layer::FCu),
        Pad::new_smd("MP2", "GND", Point::new(2.80, 3.875), PadShape::Rect { width: 1.20, height: 1.80 }, Layer::FCu),
    ];

    let silkscreen_lines = vec![
        (Point::new(-3.0, 0.325), Point::new(3.0, 0.325)),
        (Point::new(3.0, 0.325), Point::new(3.0, 4.575)),
        (Point::new(3.0, 4.575), Point::new(-3.0, 4.575)),
        (Point::new(-3.0, 4.575), Point::new(-3.0, 0.325)),
    ];

    Footprint::new(
        reference,
        value,
        "JST_SH_SM04B-SRSS-TB_1x04_P1.00mm_Horizontal",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 16. WS2812B Addressable RGB LED Footprint
// ============================================================================

/// Generates a WS2812B 5.0 x 5.0 mm SMD addressable RGB LED footprint.
pub fn ws2812b_5050(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pad_w = 1.10;
    let pad_h = 0.90;
    let dx = 2.40;
    let dy = 1.60;

    let pads = vec![
        Pad::new_smd("1", "VDD", Point::new(-dx, -dy), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
        Pad::new_smd("2", "DOUT", Point::new(-dx, dy), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
        Pad::new_smd("3", "VSS", Point::new(dx, dy), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
        Pad::new_smd("4", "DIN", Point::new(dx, -dy), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
    ];

    let silkscreen_lines = vec![
        (Point::new(-2.5, -2.5), Point::new(2.5, -2.5)),
        (Point::new(2.5, -2.5), Point::new(2.5, 2.5)),
        (Point::new(2.5, 2.5), Point::new(-2.5, 2.5)),
        (Point::new(-2.5, 2.5), Point::new(-2.5, -2.5)),
        (Point::new(-2.5, -1.0), Point::new(-1.0, -2.5)),
    ];

    Footprint::new(
        reference,
        value,
        "LED_WS2812B_PLCC4_5.0x5.0mm_P3.2mm",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 17. SOD-123 Diode Footprint
// ============================================================================

/// Generates a SOD-123 surface mount diode footprint (e.g. Schottky power path ORing diode).
pub fn diode_sod123(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pad_w = 1.00;
    let pad_h = 0.90;
    let x_off = 1.60;

    let pads = vec![
        Pad::new_smd("1", "K", Point::new(-x_off, 0.0), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
        Pad::new_smd("2", "A", Point::new(x_off, 0.0), PadShape::Rect { width: pad_w, height: pad_h }, Layer::FCu),
    ];

    let silkscreen_lines = vec![
        (Point::new(-1.0, -0.8), Point::new(1.0, -0.8)),
        (Point::new(1.0, -0.8), Point::new(1.0, 0.8)),
        (Point::new(1.0, 0.8), Point::new(-1.0, 0.8)),
        (Point::new(-1.0, 0.8), Point::new(-1.0, -0.8)),
        (Point::new(-0.6, -0.8), Point::new(-0.6, 0.8)),
    ];

    Footprint::new(
        reference,
        value,
        "D_SOD-123",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 18. Expansion Headers (1x16 and 1x20)
// ============================================================================

/// Generates a 1x10 0.1" (2.54 mm) through-hole expansion header.
pub fn header_1x10(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pin_names: Vec<String> = (1..=10).map(|n| n.to_string()).collect();
    let name_refs: Vec<&str> = pin_names.iter().map(|s| s.as_str()).collect();
    header_1xn_with_pin_names(reference, value, 10, &name_refs)
}

/// Generates a 1x16 0.1" (2.54 mm) through-hole expansion header.
pub fn header_1x16(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pin_names: Vec<String> = (1..=16).map(|n| n.to_string()).collect();
    let name_refs: Vec<&str> = pin_names.iter().map(|s| s.as_str()).collect();
    header_1xn_with_pin_names(reference, value, 16, &name_refs)
}

/// Generates a 1x20 0.1" (2.54 mm) through-hole expansion header (Raspberry Pi Pico standard).
pub fn header_1x20(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pin_names: Vec<String> = (1..=20).map(|n| n.to_string()).collect();
    let name_refs: Vec<&str> = pin_names.iter().map(|s| s.as_str()).collect();
    header_1xn_with_pin_names(reference, value, 20, &name_refs)
}

fn header_1xn_with_pin_names(
    reference: impl Into<String>,
    value: impl Into<String>,
    pin_count: usize,
    pin_names: &[&str],
) -> Footprint {
    let pitch = 2.54;
    let drill = 1.00;
    let pad_dia = 1.70;

    let mut pads = Vec::with_capacity(pin_count);

    for i in 0..pin_count {
        let pin_num = (i + 1).to_string();
        let name = match pin_names.get(i) {
            Some(&n) => n.to_string(),
            None => pin_num.clone(),
        };
        let y_pos = (i as f64) * pitch;
        let pos = Point::new(0.0, y_pos);

        let shape = if i == 0 {
            PadShape::Rect { width: pad_dia, height: pad_dia }
        } else {
            PadShape::Circle { diameter: pad_dia }
        };

        pads.push(Pad::new_tht(pin_num, name, pos, shape, drill, pad_dia));
    }

    let half_w = 1.27;
    let y_min = -1.27;
    let y_max = ((pin_count - 1) as f64) * pitch + 1.27;
    let silkscreen_lines = vec![
        (Point::new(-half_w, y_min), Point::new(half_w, y_min)),
        (Point::new(half_w, y_min), Point::new(half_w, y_max)),
        (Point::new(half_w, y_max), Point::new(-half_w, y_max)),
        (Point::new(-half_w, y_max), Point::new(-half_w, y_min)),
    ];

    Footprint::new(
        reference,
        value,
        format!("PinHeader_1x{}_P2.54mm_Vertical", pin_count),
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 19. 0805 Inductor / Ferrite Bead Footprint
// ============================================================================

/// Generates an IPC-7351B 0805 surface-mount inductor / ferrite bead footprint.
pub fn inductor_0805(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    passive_0805_internal(reference, value, "L_0805_2012Metric", false)
}

// ============================================================================
// 20. Gowin FPGA QFN-88 Footprint (GW1N-9 / GW1NR-9)
// ============================================================================

/// Generates a QFN-88 (10x10 mm body, 0.40 mm pitch) footprint with center thermal ground pad.
pub fn qfn88(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let mut pads = Vec::with_capacity(89);
    let pitch = 0.40;
    let pad_len = 0.70;
    let pad_w = 0.20;
    let radius = 5.00;

    let mut offsets = [0.0f64; 22];
    for i in 0..22 {
        offsets[i] = (-10.5 + i as f64) * pitch;
    }

    // Left side (Pins 1..22, top to bottom)
    for i in 0..22 {
        pads.push(Pad::new_smd(
            (i + 1).to_string(),
            format!("IO_{}", i + 1),
            Point::new(-radius, offsets[i]),
            PadShape::Rect { width: pad_len, height: pad_w },
            Layer::FCu,
        ));
    }

    // Bottom side (Pins 23..44, left to right)
    for i in 0..22 {
        pads.push(Pad::new_smd(
            (i + 23).to_string(),
            format!("IO_{}", i + 23),
            Point::new(offsets[i], radius),
            PadShape::Rect { width: pad_w, height: pad_len },
            Layer::FCu,
        ));
    }

    // Right side (Pins 45..66, bottom to top)
    for i in 0..22 {
        pads.push(Pad::new_smd(
            (i + 45).to_string(),
            format!("IO_{}", i + 45),
            Point::new(radius, offsets[21 - i]),
            PadShape::Rect { width: pad_len, height: pad_w },
            Layer::FCu,
        ));
    }

    // Top side (Pins 67..88, right to left)
    for i in 0..22 {
        pads.push(Pad::new_smd(
            (i + 67).to_string(),
            format!("IO_{}", i + 67),
            Point::new(offsets[21 - i], -radius),
            PadShape::Rect { width: pad_w, height: pad_len },
            Layer::FCu,
        ));
    }

    // Pin 89: Center exposed ground thermal pad (EPAD)
    pads.push(Pad::new_smd(
        "89",
        "GND_EPAD",
        Point::zero(),
        PadShape::Rect { width: 5.40, height: 5.40 },
        Layer::FCu,
    ));

    let silkscreen_lines = vec![
        (Point::new(-5.0, -5.0), Point::new(5.0, -5.0)),
        (Point::new(5.0, -5.0), Point::new(5.0, 5.0)),
        (Point::new(5.0, 5.0), Point::new(-5.0, 5.0)),
        (Point::new(-5.0, 5.0), Point::new(-5.0, -5.0)),
        (Point::new(-5.0, -4.0), Point::new(-6.0, -5.0)),
    ];

    Footprint::new(
        reference,
        value,
        "QFN-88-1EP_10x10mm_P0.4mm",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}

// ============================================================================
// 21. Hirose FX8-50P-SV 50-Pin Board-to-Board Receptacle Footprint
// ============================================================================

/// Generates a Hirose FX8-50P-SV 50-pin (2x25, 0.60 mm pitch) high-speed mezzanine connector.
pub fn hirose_fx8_50p(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let mut pads = Vec::with_capacity(52);
    let pitch = 0.60;
    let pad_w = 0.35;
    let pad_h = 0.90;
    let row_y = 1.50;

    // Row 1 (Pins 1..25, Top row, left to right)
    for i in 0..25 {
        let x = (-12.0 + i as f64) * pitch;
        pads.push(Pad::new_smd(
            (i + 1).to_string(),
            (i + 1).to_string(),
            Point::new(x, -row_y),
            PadShape::Rect { width: pad_w, height: pad_h },
            Layer::FCu,
        ));
    }

    // Row 2 (Pins 26..50, Bottom row, right to left)
    for i in 0..25 {
        let x = (12.0 - i as f64) * pitch;
        pads.push(Pad::new_smd(
            (i + 26).to_string(),
            (i + 26).to_string(),
            Point::new(x, row_y),
            PadShape::Rect { width: pad_w, height: pad_h },
            Layer::FCu,
        ));
    }

    // Retention tabs / shield brackets at ends
    pads.push(Pad::new_smd(
        "SH1",
        "GND_TAB",
        Point::new(-8.60, 0.0),
        PadShape::Rect { width: 1.20, height: 1.80 },
        Layer::FCu,
    ));
    pads.push(Pad::new_smd(
        "SH2",
        "GND_TAB",
        Point::new(8.60, 0.0),
        PadShape::Rect { width: 1.20, height: 1.80 },
        Layer::FCu,
    ));

    let silkscreen_lines = vec![
        (Point::new(-9.50, -2.20), Point::new(9.50, -2.20)),
        (Point::new(9.50, -2.20), Point::new(9.50, 2.20)),
        (Point::new(9.50, 2.20), Point::new(-9.50, 2.20)),
        (Point::new(-9.50, 2.20), Point::new(-9.50, -2.20)),
        (Point::new(-9.50, -1.50), Point::new(-10.20, -2.20)), // Pin 1 index notch
    ];

    Footprint::new(
        reference,
        value,
        "Hirose_FX8-50P-SV_2x25_P0.60mm_Vertical",
        Point::zero(),
        0.0,
        pads,
        silkscreen_lines,
    )
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tqfp32_geometry_and_counts() {
        let fp = tqfp32("U1", "ATmega328P-AU");
        assert_eq!(fp.pads.len(), 32);
        assert_eq!(fp.package, "TQFP-32_7x7mm_P0.8mm");

        let p1 = fp.pad("1").expect("Pin 1 exists");
        assert!((p1.relative_pos.x - -4.25).abs() < 1e-6);
        assert!((p1.relative_pos.y - -2.80).abs() < 1e-6);
        assert_eq!(p1.pin_name, "PD3");

        let p9 = fp.pad("9").expect("Pin 9 exists");
        assert!((p9.relative_pos.x - -2.80).abs() < 1e-6);
        assert!((p9.relative_pos.y - 4.25).abs() < 1e-6);
        assert_eq!(p9.pin_name, "PD5");
    }

    #[test]
    fn test_footprint_rotation_transformation() {
        let fp = resistor_0805("R1", "10k")
            .at_position(Point::new(50.0, 50.0))
            .with_rotation(90.0);

        let p1_world = fp.pad_world_pos("1").expect("Pin 1 world pos");
        assert!((p1_world.x - 50.0).abs() < 1e-6);
        assert!((p1_world.y - 49.05).abs() < 1e-6);

        let p2_world = fp.pad_world_pos("2").expect("Pin 2 world pos");
        assert!((p2_world.x - 50.0).abs() < 1e-6);
        assert!((p2_world.y - 50.95).abs() < 1e-6);
    }

    #[test]
    fn test_through_hole_annular_rings() {
        let header = header_1x15("J1", "EXPANSION");
        assert_eq!(header.pads.len(), 15);

        for pad in &header.pads {
            let ring = pad.annular_ring().expect("Annular ring exists");
            assert!(ring >= 0.25, "Annular ring {} mm is less than 0.25 mm", ring);
            assert_eq!(pad.pad_type.drill_diameter(), Some(1.00));
        }

        assert!(matches!(header.pads[0].shape, PadShape::Rect { .. }));
        assert!(matches!(header.pads[1].shape, PadShape::Circle { .. }));
    }

    #[test]
    fn test_usb_c_hybrid_pads() {
        let usb = usb_c_16pin("J1", "USB-C");
        assert_eq!(usb.pads.len(), 16);

        let smd_count = usb.pads.iter().filter(|p| p.pad_type.is_smd()).count();
        let tht_count = usb.pads.iter().filter(|p| p.pad_type.is_through_hole()).count();
        assert_eq!(smd_count, 12);
        assert_eq!(tht_count, 4);
    }
}

/// RT6150B WDFN-10L 2.5x2.5 (DS6150A/B-06 pp.2,13), including GND exposed pad.
/// Nominal engineering lands: 0.5mm pitch, 0.6x0.25mm peripheral pads;
/// EP 1.2x2.0mm. Paste segmentation/thermal vias remain manufacturing qualification.
pub fn rt6150b_wdfn10(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let names = [
        "VOUT", "LX2", "GND", "LX1", "VIN", "EN", "PS", "VINA", "GND", "FB",
    ];
    let mut pads = Vec::new();
    for (i, name) in names.iter().enumerate() {
        let (x, y) = if i < 5 {
            (-1.2, -1.0 + i as f64 * 0.5)
        } else {
            (1.2, 1.0 - (i - 5) as f64 * 0.5)
        };
        pads.push(Pad::new_smd(
            (i + 1).to_string(),
            *name,
            Point::new(x, y),
            PadShape::Rect {
                width: 0.6,
                height: 0.25,
            },
            Layer::FCu,
        ));
    }
    pads.push(Pad::new_smd(
        "11",
        "EP_GND",
        Point::zero(),
        PadShape::Rect {
            width: 1.2,
            height: 2.0,
        },
        Layer::FCu,
    ));
    Footprint::new(
        reference,
        value,
        "WDFN-10-1EP_2.5x2.5mm_P0.5mm_EP1.2x2.0mm",
        Point::zero(),
        0.0,
        pads,
        vec![
            (Point::new(-1.25, -1.4), Point::new(1.25, -1.4)),
            (Point::new(-1.25, 1.4), Point::new(1.25, 1.4)),
        ],
    )
}

/// DMG2305UX P-MOSFET: pin1 gate, pin2 source, pin3 drain (Diodes DS36196).
/// Manufacturer suggested SOT23 lands: 0.8x0.9mm, centers X +/-0.95/Y +/-1.0mm.
pub fn dmg2305ux_sot23(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let mut f = sot23(reference, value);
    for (pad, name) in f.pads.iter_mut().zip(["G", "S", "D"]) {
        pad.pin_name = name.into();
        pad.relative_pos.y = pad.relative_pos.y.signum();
        pad.shape = PadShape::Rect {
            width: 0.8,
            height: 0.9,
        };
    }
    f
}

/// DMG1012T SOT523, manufacturer DS31783 Rev8-2 p6 suggested pads.
/// Centers: X +/-0.5mm, Y +/-0.645mm; pads 0.40x0.51mm; G/S/D=1/2/3.
pub fn dmg1012t_sot523(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pads = [
        ("1", "G", -0.5, 0.645),
        ("2", "S", 0.5, 0.645),
        ("3", "D", 0.0, -0.645),
    ]
    .iter()
    .map(|(pin, name, x, y)| {
        Pad::new_smd(
            *pin,
            *name,
            Point::new(*x, *y),
            PadShape::Rect {
                width: 0.40,
                height: 0.51,
            },
            Layer::FCu,
        )
    })
    .collect();
    Footprint::new(
        reference,
        value,
        "SOT-523",
        Point::zero(),
        0.0,
        pads,
        vec![
            (Point::new(-0.8, -0.4), Point::new(0.8, -0.4)),
            (Point::new(-0.8, 0.4), Point::new(0.8, 0.4)),
        ],
    )
}

/// TDK VLS252012HBX-1: source catalog p3, two 0.6x2.0mm lands separated 1.3mm.
pub fn vls252012hbx(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pads = [("1", -0.95), ("2", 0.95)]
        .iter()
        .map(|(pin, x)| {
            Pad::new_smd(
                *pin,
                *pin,
                Point::new(*x, 0.0),
                PadShape::Rect {
                    width: 0.6,
                    height: 2.0,
                },
                Layer::FCu,
            )
        })
        .collect();
    Footprint::new(
        reference,
        value,
        "L_TDK_VLS252012HBX_2.5x2.0mm",
        Point::zero(),
        0.0,
        pads,
        vec![
            (Point::new(-1.25, -1.15), Point::new(1.25, -1.15)),
            (Point::new(-1.25, 1.15), Point::new(1.25, 1.15)),
        ],
    )
}

/// Worldsemi WS2812B-V6 manufacturer datasheet V1.5 p2 recommended 5050 lands.
/// Pin map is unchanged, but the old WS2812B land dimensions are not silently reused.
pub fn ws2812b_v6_5050(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let mut f = ws2812b_5050(reference, value);
    f.package = "LED_WS2812B-V6_5050".into();
    for p in &mut f.pads {
        p.relative_pos.x = p.relative_pos.x.signum() * 2.45;
        p.relative_pos.y = p.relative_pos.y.signum() * 1.65;
        p.shape = PadShape::Rect {
            width: 1.5,
            height: 1.0,
        };
    }
    f
}

/// Three-pin SWD connector, same 2.54mm pitch/hole convention as the existing headers.
pub fn header_1x3(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    header_1xn_with_pin_names(reference, value, 3, &["SWCLK", "GND", "SWDIO"])
}


/// Espressif WROVER-E datasheet v2.4 Fig15 and retained manufacturer DXF.
/// Actual module body18x31.4mm; antenna occupies y=-15.7..-9.48.
/// The board model cannot enforce the required antenna copper/component keepout.
pub fn esp32_wrover_e(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let names = ["GND","3V3","EN","GPIO36","GPIO39","GPIO34","GPIO35","GPIO32","GPIO33","GPIO25","GPIO26","GPIO27","GPIO14","GPIO12","GND","GPIO13","NC17","NC18","NC19","NC20","NC21","NC22","GPIO15","GPIO2","GPIO0","GPIO4","NC27","NC28","GPIO5","GPIO18","GPIO19","NC32","GPIO21","RXD0","TXD0","GPIO22","GPIO23","GND"];
    let mut pads=Vec::new();
    for i in 0..19 {
        for (n,x,y) in [(i+1,-8.75,-8.26+i as f64*1.27),(38-i,8.75,-8.26+i as f64*1.27)] {
            pads.push(Pad::new_smd(n.to_string(),names[n-1],Point::new(x,y),PadShape::Rect{width:1.5,height:0.9},Layer::FCu));
        }
    }
    // Nine distinct physical sublands, all ground; avoid duplicate-number assignment loss.
    for row in 0..3 {for col in 0..3 {
        pads.push(Pad::new_smd(format!("EP{}",row*3+col+1),"GND",Point::new(-1.5+(col as f64-1.0)*1.4,-0.46+(row as f64-1.0)*1.4),PadShape::Rect{width:0.9,height:0.9},Layer::FCu));
    }}
    let silk=vec![(Point::new(-9.0,-15.7),Point::new(9.0,-15.7)),(Point::new(-9.0,-9.48),Point::new(9.0,-9.48))];
    Footprint::new(reference,value,"ESP32-WROVER-E_18x31.4mm",Point::zero(),0.0,pads,silk)
}

/// Silicon Labs CP2102N Rev1.5 section6.2 recommended land; not generic QFN.
pub fn cp2102n_qfn28(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let names=["DCD","RI","GND","D+","D-","VDD","VREGIN","VBUS","RSTb","NC","SUSPENDb","SUSPEND","CHREN","CHR1","CHR0","GPIO3","GPIO2","GPIO1","GPIO0","GPIO6","GPIO5","GPIO4","CTS","RTS","RXD","TXD","DSR","DTR"];
    let mut pads=Vec::new();
    for i in 0..7 {
        let t=-1.5+i as f64*0.5;
        for (n,x,y,w,h) in [(i+1,-2.4,t,0.95,0.30),(i+8,t,2.4,0.30,0.95),(i+15,2.4,-t,0.95,0.30),(i+22,-t,-2.4,0.30,0.95)] {
            pads.push(Pad::new_smd(n.to_string(),names[n-1],Point::new(x,y),PadShape::Rect{width:w,height:h},Layer::FCu));
        }
    }
    pads.push(Pad::new_smd("29","GND",Point::zero(),PadShape::Rect{width:3.35,height:3.35},Layer::FCu));
    // Paste windowing/mask rules remain native fabrication qualification, not represented here.
    Footprint::new(reference,value,"CP2102N_QFN28_5x5mm_P0.5mm_EP3.35",Point::zero(),0.0,pads,vec![])
}

/// Ai-Thinker OV2640 numbering with exact bundled Hirose FH12-24S land coordinates.
/// MP1/MP2 are explicit aliases of the two manufacturer MP lands.
pub fn ov2640_hirose_fh12(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let names=["Y0","Y1","Y4","Y3","Y5","Y2","Y6","PCLK","Y7","DGND","Y8","XCLK","Y9","DOVDD","DVDD","HREF","PWDN","VSYNC","RESET","SIOC","AVDD","SIOD","AGND","NC"];
    let mut pads=Vec::new();
    for i in 0..24 {pads.push(Pad::new_smd((i+1).to_string(),names[i],Point::new(-5.75+i as f64*0.5,-1.85),PadShape::Rect{width:0.3,height:1.3},Layer::FCu));}
    for (n,x) in [("MP1",-7.65),("MP2",7.65)] {pads.push(Pad::new_smd(n,"GND",Point::new(x,1.4),PadShape::Rect{width:1.8,height:2.2},Layer::FCu));}
    Footprint::new(reference,value,"Hirose_FH12-24S-0.5SH_1x24_P0.50mm",Point::zero(),0.0,pads,vec![(Point::new(-7.55,-1.2),Point::new(-7.55,3.4)),(Point::new(-7.55,3.4),Point::new(7.55,3.4)),(Point::new(7.55,3.4),Point::new(7.55,-1.2))])
}

/// Pinned KiCad10 Molex104031-0811 lands; detect9/10 intentionally unused.
pub fn microsd_molex_104031(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let signals=[("1","DAT2",-3.105,-5.45,0.85,1.1),("2","DAT3",-2.005,-5.45,0.85,1.1),("3","CMD",-0.905,-5.45,0.85,1.1),("4","VDD",0.195,-5.45,0.85,1.1),("5","CLK",1.295,-5.45,0.85,1.1),("6","VSS",2.395,-5.45,0.85,1.1),("7","DAT0",3.495,-5.45,0.85,1.1),("8","DAT1",4.545,-5.45,0.75,1.1),("9","DETECT",-5.74,0.7,1.2,1.0),("10","DETECT_RETURN",-5.74,4.4,1.2,1.0),("SH1","GND",-5.565,-5.325,1.55,1.35),("SH2","GND",-2.24,5.375,1.9,1.35),("SH3","GND",3.73,5.375,1.9,1.35),("SH4","GND",5.755,-5.1,1.17,1.8)];
    let pads=signals.into_iter().map(|(n,s,x,y,w,h)|Pad::new_smd(n,s,Point::new(x,y),PadShape::Rect{width:w,height:h},Layer::FCu)).collect();
    Footprint::new(reference,value,"microSD_HC_Molex_104031-0811",Point::zero(),0.0,pads,vec![(Point::new(-6.4,-6.0),Point::new(6.4,-6.0)),(Point::new(6.4,-6.0),Point::new(6.4,6.05)),(Point::new(6.4,6.05),Point::new(-6.4,6.05))])
}

/// C&K PTS636 straight gullwing H2.5 source-library geometry, two terminals.
/// Exact assembled actuator/order variant remains a sourcing qualification hold.
pub fn pts636_smd(reference: impl Into<String>, value: impl Into<String>) -> Footprint {
    let pads=[("1",-3.875),("2",3.875)].into_iter().map(|(n,x)|Pad::new_smd(n,n,Point::new(x,0.0),PadShape::Rect{width:1.25,height:1.0},Layer::FCu)).collect();
    Footprint::new(reference,value,"SW_CK_PTS636_6x3.5mm_SMD",Point::zero(),0.0,pads,vec![(Point::new(-3.0,-1.75),Point::new(3.0,-1.75)),(Point::new(3.0,-1.75),Point::new(3.0,1.75)),(Point::new(3.0,1.75),Point::new(-3.0,1.75)),(Point::new(-3.0,1.75),Point::new(-3.0,-1.75))])
}
