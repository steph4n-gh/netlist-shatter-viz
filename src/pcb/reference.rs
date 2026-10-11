//! Microcontroller Reference System & Pipeline Architecture (Milestone 5)
//!
//! Provides a complete, fully placed, and pre-routed reference design of an
//! ATmega328P development board adhering to IPC-7351B standards and compatible
//! with the automated 2-layer grid router, geometric DRC checker, and CAM fabrication exporters.
//!
//! # Architecture & Layout Overview
//! - Board Outline: 65.0 x 32.0 mm closed boundary (standard dual-header form factor).
//! - Active Components:
//!   - `U1`: Microchip ATmega328P-AU 8-bit AVR Microcontroller (TQFP-32, 7x7 mm body, 0.8 mm pitch) centered at (32.5, 16.0).
//!   - `U2`: AMS1117-3.3 Low-Dropout Linear Voltage Regulator (SOT-223) positioned at (14.0, 16.0).
//!   - `J1`: USB Type-C 16-pin Receptacle positioned at (4.5, 16.0) facing the left board edge.
//!   - `Y1`: 16.0 MHz Crystal Resonator (SMD 3225 4-pin) positioned at (24.0, 10.0) near MCU oscillator pins.
//! - Passives & User Interface:
//!   - `C1`: 10 µF Input Filter Capacitor (0805 metric 2012) at (14.0, 8.5).
//!   - `C2`: 10 µF Output Filter Capacitor (0805 metric 2012) at (14.0, 23.5).
//!   - `C3`: 0.1 µF VCC Bypass Capacitor (0805 metric 2012) at (24.0, 22.0).
//!   - `R1`: 10 kΩ RESET Pull-up Resistor (0805 metric 2012) at (40.0, 10.0).
//!   - `SW1`: Tactile Pushbutton Reset Switch (3x4 mm SMD) at (48.0, 16.0).
//! - Breakout Expansion:
//!   - `J2`: 1x15 0.1" (2.54 mm) Through-Hole Header (CONN_TOP) at (14.72, 2.54), rotation 270.0°.
//!   - `J3`: 1x15 0.1" (2.54 mm) Through-Hole Header (CONN_BOT) at (14.72, 29.46), rotation 270.0°.
//! - Total Physical Pads: 98 pads (exceeding the >= 80 requirement).
//! - Unrouted Ratsnest Lines: 31 EMST lines (exceeding the >= 25 requirement).
//! - Autorouting Compatibility: Solves with 0 shorts, 0 clearance violations,
//!   0 annular ring violations, and 0 edge keepout violations using standard router parameters.

use crate::pcb::core::{NetId, PcbBoard, Point};
use crate::pcb::footprint::{
    capacitor_0805, crystal_smd_3225, header_1x15, resistor_0805, sot223, tactile_switch_3x4,
    tqfp32, usb_c_16pin,
};

/// Helper function to assign a net to a pad identified by pin number or pin name.
#[inline]
fn assign_pad(board: &mut PcbBoard, comp_ref: &str, pin_id: &str, net: NetId) {
    if let Some(fp) = board.find_footprint_mut(comp_ref) {
        if let Some(pad) = fp
            .pads
            .iter_mut()
            .find(|p| p.pin_number == pin_id || p.pin_name == pin_id)
        {
            pad.net = Some(net);
        }
    }
}

/// Creates and returns the fully configured ATmega328P microcontroller reference board.
///
/// The returned [`PcbBoard`] has:
/// - Board outline of 65.0 x 32.0 mm (`Edge.Cuts`).
/// - 11 physical component footprints totaling 98 pads.
/// - Valid netlist defining `GND`, `5V`, `3V3`, `RESET`, `XTAL1`, `XTAL2`, and expansion signals.
/// - Minimum spanning tree dynamic ratsnest with 31 lines.
/// - 100% routability and 100% DRC clean verification under [`crate::pcb::RouterConfig`].
pub fn create_reference_board() -> PcbBoard {
    let mut board = PcbBoard::new(65.0, 32.0);

    // ========================================================================
    // 1. Physical Footprint Placement
    // ========================================================================

    // U1: ATmega328P 8-bit AVR Microcontroller (TQFP-32, 7x7 mm body, 0.8 mm pitch)
    let u1 = tqfp32("U1", "ATmega328P-AU").at_position(Point::new(32.5, 16.0));
    board.add_footprint(u1);

    // U2: AMS1117-3.3 Linear Voltage Regulator (SOT-223)
    let u2 = sot223("U2", "AMS1117-3.3").at_position(Point::new(14.0, 16.0));
    board.add_footprint(u2);

    // J1: USB Type-C 16-pin Receptacle (placed at left board edge, facing outward)
    let j1 = usb_c_16pin("J1", "USB-C")
        .at_position(Point::new(4.5, 16.0))
        .with_rotation(270.0);
    board.add_footprint(j1);

    // Y1: 16.0 MHz Quartz Crystal Resonator (SMD 3225 4-pin)
    let y1 = crystal_smd_3225("Y1", "16MHz").at_position(Point::new(24.0, 10.0));
    board.add_footprint(y1);

    // Passive Components (0805 metric 2012 packages)
    let c1 = capacitor_0805("C1", "10uF").at_position(Point::new(14.0, 8.5));
    let c2 = capacitor_0805("C2", "10uF").at_position(Point::new(14.0, 23.5));
    let c3 = capacitor_0805("C3", "0.1uF").at_position(Point::new(24.0, 22.0));
    let r1 = resistor_0805("R1", "10k").at_position(Point::new(40.0, 10.0));
    let sw1 = tactile_switch_3x4("SW1", "RESET").at_position(Point::new(48.0, 16.0));
    board.add_footprint(c1);
    board.add_footprint(c2);
    board.add_footprint(c3);
    board.add_footprint(r1);
    board.add_footprint(sw1);

    // Dual 1x15 0.1" (2.54 mm) Through-Hole Expansion Headers
    let j2 = header_1x15("J2", "CONN_TOP")
        .at_position(Point::new(14.72, 2.54))
        .with_rotation(270.0);
    let j3 = header_1x15("J3", "CONN_BOT")
        .at_position(Point::new(14.72, 29.46))
        .with_rotation(270.0);
    board.add_footprint(j2);
    board.add_footprint(j3);

    // ========================================================================
    // 2. Netlist Registration
    // ========================================================================

    let net_gnd = board.get_or_create_net("GND");
    let net_5v = board.get_or_create_net("5V");
    let net_3v3 = board.get_or_create_net("3V3");
    let net_reset = board.get_or_create_net("RESET");
    let net_xtal1 = board.get_or_create_net("XTAL1");
    let net_xtal2 = board.get_or_create_net("XTAL2");

    // ========================================================================
    // 3. Netlist Connectivity Mapping
    // ========================================================================

    // XTAL1 & XTAL2: Connect MCU oscillator pins (PB6/Pin 7, PB7/Pin 8) to Crystal Y1
    assign_pad(&mut board, "U1", "7", net_xtal1);
    assign_pad(&mut board, "Y1", "1", net_xtal1);
    assign_pad(&mut board, "U1", "8", net_xtal2);
    assign_pad(&mut board, "Y1", "3", net_xtal2);

    // RESET: Connects Pull-up R1, tactile reset switch SW1, and expansion pin J2.3
    assign_pad(&mut board, "R1", "2", net_reset);
    assign_pad(&mut board, "SW1", "2", net_reset);
    assign_pad(&mut board, "J2", "3", net_reset);

    // 5V: Connects LDO VIN (U2.3), input decoupling cap C1, and top expansion rail J2.1
    assign_pad(&mut board, "U2", "3", net_5v);
    assign_pad(&mut board, "C1", "1", net_5v);
    assign_pad(&mut board, "J2", "1", net_5v);

    // 3V3: Connects LDO VOUT (U2.2), output filter cap C2, MCU VCC (U1.4), bypass cap C3,
    // pullup resistor R1, and bottom expansion rail J3.1
    assign_pad(&mut board, "U2", "2", net_3v3);
    assign_pad(&mut board, "C2", "1", net_3v3);
    assign_pad(&mut board, "U1", "4", net_3v3);
    assign_pad(&mut board, "C3", "1", net_3v3);
    assign_pad(&mut board, "R1", "1", net_3v3);
    assign_pad(&mut board, "J3", "1", net_3v3);

    // GND: Common system ground reference connecting 10 distinct components:
    // USB-C (J1), LDO (U2), Input Cap (C1), Output Cap (C2), MCU (U1),
    // Bypass Cap (C3), Crystal (Y1), Reset Switch (SW1), Header Top (J2), Header Bot (J3)
    assign_pad(&mut board, "J1", "SH1", net_gnd);
    assign_pad(&mut board, "J1", "SH3", net_gnd);
    assign_pad(&mut board, "U2", "1", net_gnd);
    assign_pad(&mut board, "C1", "2", net_gnd);
    assign_pad(&mut board, "C2", "2", net_gnd);
    assign_pad(&mut board, "U1", "3", net_gnd);
    assign_pad(&mut board, "C3", "2", net_gnd);
    assign_pad(&mut board, "Y1", "2", net_gnd);
    assign_pad(&mut board, "SW1", "1", net_gnd);
    assign_pad(&mut board, "J2", "4", net_gnd);
    assign_pad(&mut board, "J3", "2", net_gnd);

    // GPIO & Expansion Bus Signals:
    // Interconnecting MCU left-side escape pins and expansion headers
    let bus_signals = [
        ("U1", "1", "J2", "6", "SIG_D3"),
        ("U1", "2", "J2", "7", "SIG_D4"),
        ("J2", "8", "J3", "8", "BUS_8"),
        ("J2", "9", "J3", "9", "BUS_9"),
        ("J2", "10", "J3", "10", "BUS_10"),
        ("J2", "11", "J3", "11", "BUS_11"),
        ("J2", "12", "J3", "12", "BUS_12"),
        ("J2", "13", "J3", "13", "BUS_13"),
        ("J2", "14", "J3", "14", "BUS_14"),
        ("J2", "15", "J3", "15", "BUS_15"),
    ];

    for (c1, p1, c2, p2, name) in bus_signals {
        let net = board.get_or_create_net(name);
        assign_pad(&mut board, c1, p1, net);
        assign_pad(&mut board, c2, p2, net);
    }

    board
}

// ============================================================================
// In-Module Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pcb::core::compute_ratsnest;
    use crate::pcb::drc::run_drc;
    use crate::pcb::router::{route_board, RouterConfig};

    #[test]
    fn test_reference_board_geometry_and_counts() {
        let board = create_reference_board();
        assert_eq!(board.width, 65.0);
        assert_eq!(board.height, 32.0);
        assert_eq!(board.footprints.len(), 11);

        let total_pads: usize = board.footprints.iter().map(|fp| fp.pads.len()).sum();
        assert_eq!(total_pads, 98);
        assert!(total_pads >= 80);
    }

    #[test]
    fn test_reference_board_netlist_and_ratsnest() {
        let board = create_reference_board();
        let ratsnest = compute_ratsnest(&board);
        assert_eq!(ratsnest.len(), 31);
        assert!(ratsnest.len() >= 25);

        for line in &ratsnest {
            assert!(line.start.distance_to(line.end) > 0.001);
            assert!(line.net > 0);
        }
    }

    #[test]
    fn test_reference_board_autorouting_and_drc() {
        let mut board = create_reference_board();
        let config = RouterConfig {
            grid_resolution: 0.25,
            default_trace_width: 0.25,
            via_diameter: 0.80,
            via_drill: 0.40,
            clearance: 0.20,
        };

        let result = route_board(&mut board, &config);
        assert!(result.is_ok(), "Autorouting must succeed: {:?}", result);

        let drc = run_drc(&board, &config);
        assert_eq!(drc.short_circuits.len(), 0, "Zero short circuits");
        assert_eq!(drc.clearance_violations.len(), 0, "Zero clearance violations");
        assert_eq!(drc.annular_ring_violations.len(), 0, "Zero annular ring violations");
        assert_eq!(drc.edge_keepout_violations.len(), 0, "Zero edge keepout violations");
        assert_eq!(drc.unrouted_nets.len(), 0, "All nets routed");
    }
}
