//! Framework Card engineering draft.
//! See docs/DESIGN_REVIEW.md and generated report.json for known circuit blockers.
//! Routing success is not proof of electrical correctness or manufacturing qualification.

use crate::pcb::assembly::{BomEntry, CplEntry};
use crate::pcb::core::{NetId, PcbBoard, Point};
use crate::pcb::footprint::{
    ap2112k_33, capacitor_0805, crystal_smd_3225, header_1x10, jst_sh_4pin, led_0805, qfn56, resistor_0805,
    soic8, tactile_switch_3x4, usb_c_16pin,
};
use crate::pcb::router::{route_board, RouterConfig, RoutingError};

/// Helper to assign a net to a pad identified by pin number or pin name.
#[inline]
fn assign_pad(board: &mut PcbBoard, comp_ref: &str, pin_id: &str, net: NetId) {
    let net_name=board.get_net_name(net).expect("Unknown net").to_owned();
    board.assign_pad_net(comp_ref,pin_id,&net_name).expect("Invalid board netlist");
}

/// Creates the Framework RP2040 circuit draft for routing and design review.
///
/// Board Specs:
/// - Dimensions: 26.0 x 30.0 mm closed boundary (`Edge.Cuts`)
/// - 0.8 mm PCB thickness
/// - 166 physical pads across 35 components
/// - The generic USB receptacle still needs the official plug and mechanical design.
pub fn create_framework_card_board() -> PcbBoard {
    let mut board = PcbBoard::new(26.0, 30.0);
    board.thickness = 0.8;

    // ========================================================================
    // 1. Physical Footprint Placement
    // ========================================================================

    // J1: Generic USB Type-C receptacle; not yet the qualified Framework plug.
    let j1 = usb_c_16pin("J1", "USB-C")
        .at_position(Point::new(13.0, 3.5));
    board.add_footprint(j1);

    // CC Pull-down resistors (5.1k, 0805)
    let r_cc1 = resistor_0805("R1", "5.1k").at_position(Point::new(7.0, 10.5));
    let r_cc2 = resistor_0805("R2", "5.1k").at_position(Point::new(19.0, 10.2));
    board.add_footprint(r_cc1);
    board.add_footprint(r_cc2);

    // USB Damping Resistors (27Ω, 0805)
    let r_usb1 = resistor_0805("R3", "27R").at_position(Point::new(15.0, 9.5));
    let r_usb2 = resistor_0805("R4", "27R").at_position(Point::new(11.0, 9.5));
    board.add_footprint(r_usb1);
    board.add_footprint(r_usb2);

    // U3: AP2112K-3.3V 600mA Low-Dropout Regulator (SOT-23-5)
    let u3 = ap2112k_33("U3").at_position(Point::new(4.5, 7.0));
    let c1 = capacitor_0805("C1", "10uF")
        .at_position(Point::new(11.5, 6.5))
        .with_rotation(90.0);
    let c2 = capacitor_0805("C2", "10uF").at_position(Point::new(4.5, 10.0));
    board.add_footprint(u3);
    board.add_footprint(c1);
    board.add_footprint(c2);

    // U1: Raspberry Pi RP2040 (QFN-56, 7x7mm body) centered at X=13.0, Y=15.0
    let u1 = qfn56("U1", "RP2040").at_position(Point::new(13.0, 15.0));
    board.add_footprint(u1);

    // U2: Winbond W25Q128 16MB QSPI Flash (SOIC-8) at X=21.5, Y=14.0
    let u2 = soic8("U2", "W25Q128JVS").at_position(Point::new(21.5, 14.0));
    board.add_footprint(u2);

    // Preserve the signal pin order while reserving the actual JST mounting body.
    let j2 = jst_sh_4pin("J2", "SM04B-SRSS-TB").at_position(Point::new(3.8, 16.114999));
    board.add_footprint(j2);

    // Y1: 12.0 MHz Crystal Resonator (SMD 3225 4-pin) at X=13.0, Y=22.0 (rotated 90°)
    let y1 = crystal_smd_3225("Y1", "ABM8-272-T3")
        .at_position(Point::new(13.0, 22.0))
        .with_rotation(90.0);
    board.add_footprint(y1);

    // C3: 1uF 1.1V Core Decoupling Capacitor at X=4.5, Y=13.0
    let c3 = capacitor_0805("C3", "1uF").at_position(Point::new(4.5, 13.0));
    board.add_footprint(c3);

    // SW1: Tactile BOOTSEL Pushbutton Switch (3x4mm SMD) at X=21.5, Y=19.5 (adjacent to Flash CS)
    let sw1 = tactile_switch_3x4("SW1", "BOOTSEL").at_position(Point::new(21.5, 19.5));
    board.add_footprint(sw1);

    // SW2: Tactile RESET Pushbutton Switch (3x4mm SMD) at X=21.5, Y=23.5 (near RP2040 RUN pin)
    let sw2 = tactile_switch_3x4("SW2", "RESET").at_position(Point::new(21.5, 23.5));
    board.add_footprint(sw2);

    // LED1: 0805 User Status LED at X=8.5, Y=22.0 (facing RP2040 GPIO15)
    let led1 = led_0805("LED1", "STATUS").at_position(Point::new(8.5, 22.0));
    board.add_footprint(led1);

    // HDR1: 1x10 0.1" (2.54mm) Expansion Header along bottom edge at Y=27.5
    let hdr1 = header_1x10("HDR1", "EXPANSION")
        .at_position(Point::new(1.57, 27.5))
        .with_rotation(270.0);
    board.add_footprint(hdr1);

    // ========================================================================
    // 2. Net Registration
    // ========================================================================

    let net_gnd = board.get_or_create_net("GND");
    let net_vbus = board.get_or_create_net("5V_VBUS");
    let net_3v3 = board.get_or_create_net("3V3");
    let net_1v1 = board.get_or_create_net("1V1_CORE");
    let net_usb_dp = board.get_or_create_net("USB_DP");
    let net_usb_dm = board.get_or_create_net("USB_DM");
    let net_qspi_clk = board.get_or_create_net("QSPI_CLK");
    let net_qspi_cs = board.get_or_create_net("QSPI_CS");
    let net_qspi_d0 = board.get_or_create_net("QSPI_D0");
    let net_qspi_d1 = board.get_or_create_net("QSPI_D1");
    let net_xtal_in = board.get_or_create_net("XTAL_IN");
    let net_xtal_out = board.get_or_create_net("XTAL_OUT");
    let net_run = board.get_or_create_net("RUN_RESET");
    let net_sda = board.get_or_create_net("I2C_SDA");
    let net_scl = board.get_or_create_net("I2C_SCL");
    let net_status_led = board.get_or_create_net("STATUS_LED");
    let net_swclk = board.get_or_create_net("SWCLK");
    let net_swdio = board.get_or_create_net("SWDIO");

    // ========================================================================
    // 3. Netlist Connectivity
    // ========================================================================

    // Power: 5V VBUS from USB-C
    assign_pad(&mut board, "J1", "A4", net_vbus);
    assign_pad(&mut board, "C1", "1", net_vbus);
    assign_pad(&mut board, "U3", "1", net_vbus); // VIN
    assign_pad(&mut board, "U3", "3", net_vbus); // EN

    // Power: 3.3V Output
    assign_pad(&mut board, "U3", "5", net_3v3); // VOUT
    assign_pad(&mut board, "C2", "1", net_3v3);
    assign_pad(&mut board, "U1", "1", net_3v3);  // IOVDD1
    assign_pad(&mut board, "U2", "8", net_3v3);  // Flash VCC
    assign_pad(&mut board, "J2", "2", net_3v3);  // Qwiic 3V3
    assign_pad(&mut board, "HDR1", "1", net_3v3);

    // Power: 1.1V Core Decoupling
    assign_pad(&mut board, "U1", "45", net_1v1);
    assign_pad(&mut board, "C3", "1", net_1v1);

    // Power: Ground
    assign_pad(&mut board, "J1", "SH1", net_gnd);
    assign_pad(&mut board, "J1", "SH3", net_gnd);
    assign_pad(&mut board, "R1", "1", net_gnd);
    assign_pad(&mut board, "R2", "2", net_gnd);
    assign_pad(&mut board, "C1", "2", net_gnd);
    assign_pad(&mut board, "C2", "2", net_gnd);
    assign_pad(&mut board, "U3", "2", net_gnd);
    assign_pad(&mut board, "U1", "57", net_gnd); // Thermal Pad (EPAD)
    assign_pad(&mut board, "U2", "4", net_gnd);  // Flash GND
    assign_pad(&mut board, "Y1", "2", net_gnd);
    assign_pad(&mut board, "Y1", "4", net_gnd);
    assign_pad(&mut board, "C3", "2", net_gnd);
    assign_pad(&mut board, "SW1", "1", net_gnd); // BOOTSEL switch to GND
    assign_pad(&mut board, "SW2", "1", net_gnd); // RESET switch to GND
    assign_pad(&mut board, "J2", "1", net_gnd);  // Qwiic GND
    assign_pad(&mut board, "HDR1", "2", net_gnd);
    assign_pad(&mut board, "HDR1", "10", net_gnd);

    // USB CC Pull-downs
    let net_cc1 = board.get_or_create_net("CC1");
    let net_cc2 = board.get_or_create_net("CC2");
    assign_pad(&mut board, "J1", "A5", net_cc1);
    assign_pad(&mut board, "R1", "2", net_cc1);
    assign_pad(&mut board, "J1", "B5", net_cc2);
    assign_pad(&mut board, "R2", "1", net_cc2);

    // USB Data Lines with Damping Resistors
    let net_conn_dp = board.get_or_create_net("USB_CONN_DP");
    let net_conn_dm = board.get_or_create_net("USB_CONN_DM");
    assign_pad(&mut board, "J1", "B6", net_conn_dp);
    assign_pad(&mut board, "R3", "1", net_conn_dp);
    assign_pad(&mut board, "R3", "2", net_usb_dp);
    assign_pad(&mut board, "U1", "47", net_usb_dp);

    assign_pad(&mut board, "J1", "A7", net_conn_dm);
    assign_pad(&mut board, "R4", "1", net_conn_dm);
    assign_pad(&mut board, "R4", "2", net_usb_dm);
    assign_pad(&mut board, "U1", "46", net_usb_dm);

    // QSPI Flash Interface (Winbond W25Q128)
    assign_pad(&mut board, "U1", "52", net_qspi_clk);
    assign_pad(&mut board, "U2", "6", net_qspi_clk);
    assign_pad(&mut board, "U1", "56", net_qspi_cs);
    assign_pad(&mut board, "U2", "1", net_qspi_cs);
    assign_pad(&mut board, "U1", "53", net_qspi_d0);
    assign_pad(&mut board, "U2", "5", net_qspi_d0);
    assign_pad(&mut board, "U1", "55", net_qspi_d1);
    assign_pad(&mut board, "U2", "2", net_qspi_d1);

    // Crystal Oscillator
    assign_pad(&mut board, "U1", "20", net_xtal_in);  // XIN (pin 20)
    assign_pad(&mut board, "Y1", "1", net_xtal_in);
    assign_pad(&mut board, "U1", "21", net_xtal_out); // XOUT (pin 21)

    // Controls: Reset
    assign_pad(&mut board, "U1", "26", net_run); // RUN (pin 26)
    assign_pad(&mut board, "SW2", "2", net_run);
    assign_pad(&mut board, "HDR1", "9", net_run);

    // Peripherals: I2C (Qwiic / STEMMA QT)
    assign_pad(&mut board, "U1", "6", net_sda); // GPIO4 (SDA, pin 6)
    assign_pad(&mut board, "J2", "3", net_sda);
    assign_pad(&mut board, "HDR1", "3", net_sda);

    assign_pad(&mut board, "U1", "7", net_scl); // GPIO5 (SCL, pin 7)
    assign_pad(&mut board, "J2", "4", net_scl);
    assign_pad(&mut board, "HDR1", "4", net_scl);

    // Status LED: RP2040 GPIO15 -> series resistor -> LED1 anode.
    assign_pad(&mut board, "U1", "18", net_status_led);

    // SWD Debug Header (Picoprobe)
    assign_pad(&mut board, "U1", "24", net_swclk); // SWCLK (pin 24)
    assign_pad(&mut board, "HDR1", "7", net_swclk);
    assign_pad(&mut board, "U1", "25", net_swdio); // SWDIO (pin 25)
    assign_pad(&mut board, "HDR1", "8", net_swdio);

    // General I/O on Header
    let net_gpio0 = board.get_or_create_net("GPIO0_TX");
    let net_gpio1 = board.get_or_create_net("GPIO1_RX");
    assign_pad(&mut board, "U1", "2", net_gpio0); // GPIO0 (pin 2)
    assign_pad(&mut board, "HDR1", "5", net_gpio0);
    assign_pad(&mut board, "U1", "3", net_gpio1); // GPIO1 (pin 3)
    assign_pad(&mut board, "HDR1", "6", net_gpio1);

    crate::pcb::rp2040::complete_core(&mut board);
    // Raspberry Pi's reference clock and boot circuits, at 3.3V IOVDD.
    // The existing placement helper keeps new passives out of component bodies.
    for (reference, value, position) in [
        ("R5", "1k", Point::new(15.0, 21.0)),
        ("R6", "1k", Point::new(21.5, 17.0)),
        ("R7", "1k", Point::new(8.5, 24.0)),
        ("R8", "10k", Point::new(21.5, 25.0)),
        ("C4", "15pF", Point::new(11.0, 22.0)),
        ("C5", "15pF", Point::new(15.0, 22.0)),
    ] {
        crate::pcb::rp2040::add_passive(
            &mut board,
            crate::pcb::rp2040::passive(reference, value, position, 0.0),
        );
    }
    let crystal_drive = board.get_or_create_net("XTAL_DRIVE");
    let boot_button = board.get_or_create_net("BOOT_BUTTON");
    let led_anode = board.get_or_create_net("STATUS_LED_A");
    for (reference, pin, net) in [
        ("R5", "1", net_xtal_out), ("R5", "2", crystal_drive),
        ("Y1", "3", crystal_drive),
        ("C4", "1", net_xtal_in), ("C4", "2", net_gnd),
        ("C5", "1", crystal_drive), ("C5", "2", net_gnd),
        ("R6", "1", net_qspi_cs), ("R6", "2", boot_button),
        ("SW1", "2", boot_button),
        ("R7", "1", net_status_led), ("R7", "2", led_anode),
        ("LED1", "1", net_gnd), ("LED1", "2", led_anode),
        ("R8", "1", net_3v3), ("R8", "2", net_run),
    ] {
        assign_pad(&mut board, reference, pin, net);
    }
    // The cold-routing experiment reserves room for two staggered via rows
    // around the RP2040 banks. Keep the crystal group rigid, separate bodies,
    // and align flash pads with their MCU bank; pad definitions stay unchanged.
    // Evidence: build/framework/placement-completion-07/both-bank-space-30.
    for (reference, x, y, rotation) in [
        ("J1", 13.0, 3.1, 0.0),
        ("R1", 5.325, 11.875, 0.0),
        ("R3", 15.0, 8.975, 0.0),
        ("R4", 11.0, 8.975, 0.0),
        ("C1", 1.675, 6.5, 90.0),
        ("U2", 23.125, 8.85, 270.0),
        ("Y1", 13.0, 22.925, 90.0),
        ("C3", 17.825, 19.425, 90.0),
        ("SW2", 21.5, 24.0, 0.0),
        ("C22", 13.25, 20.675, 0.0),
        ("C21", 7.700001, 18.75, 90.0),
        ("C20", 9.0, 10.0, 90.0),
        ("C25", 7.5, 9.0, 0.0),
        ("C27", 7.75, 12.0, 0.0),
        ("C31", 19.65, 7.775, 0.0),
        ("R5", 15.5, 21.925, 0.0),
        ("C4", 10.5, 24.425, 0.0),
        ("C5", 15.5, 22.925, 0.0),
    ] {
        let footprint = board.find_footprint_mut(reference).unwrap();
        footprint.position = Point::new(x, y);
        footprint.rotation = rotation;
    }
    board
}

/// Solves and returns the fully routed Framework RP2040 Pro Expansion Card board.
pub fn solve_framework_card_board(config: &RouterConfig) -> Result<PcbBoard, RoutingError> {
    let mut board = create_framework_card_board();
    route_board(&mut board, config)?;
    Ok(board)
}

/// Generates an engineering BOM from the actual circuit, including its added passives.
pub fn generate_framework_card_bom() -> Vec<BomEntry> {
    crate::pcb::assembly::generate_bom_from_board(&create_framework_card_board())
}

/// Generates the Pick-and-Place (CPL / Centroid) list for the Framework RP2040 Pro Card.
pub fn generate_framework_card_cpl(board: &PcbBoard) -> Vec<CplEntry> {
    crate::pcb::assembly::generate_cpl_from_board(board)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pcb::core::compute_ratsnest;

    #[test]
    fn test_framework_card_geometry_and_counts() {
        let board = create_framework_card_board();
        assert_eq!(board.width, 26.0);
        assert_eq!(board.height, 30.0);
        assert_eq!(board.footprints.len(), 35);

        let total_pads: usize = board.footprints.iter().map(|fp| fp.pads.len()).sum();
        assert_eq!(total_pads, 166);
    }

    #[test]
    fn test_framework_card_netlist_and_ratsnest() {
        let board = create_framework_card_board();
        let ratsnest = compute_ratsnest(&board);
        assert!(!ratsnest.is_empty(), "Ratsnest must contain lines before routing");

        assert!(board.net_names.values().any(|n| n == "GND"));
        assert!(board.net_names.values().any(|n| n == "5V_VBUS"));
        assert!(board.net_names.values().any(|n| n == "3V3"));
        assert!(board.net_names.values().any(|n| n == "USB_DP"));
        assert!(board.net_names.values().any(|n| n == "QSPI_CLK"));
        assert!(board.net_names.values().any(|n| n == "RUN_RESET"));
        assert!(board.net_names.values().any(|n| n == "I2C_SDA"));
        assert!(board.net_names.values().any(|n| n == "SWCLK"));
    }

    #[test]
    fn test_framework_placement_leaves_both_mcu_via_banks_open() {
        let board = create_framework_card_board();
        assert!(crate::pcb::placement_issues(&board).is_empty());
        let bounds = |reference: &str| {
            let fp = board.find_footprint(reference).unwrap();
            fp.pads.iter().map(|p| fp.pad_world_bbox(p))
                .reduce(|a, b| a.union(&b)).unwrap()
        };
        let mcu = bounds("U1");
        // Two staggered rows of the existing 0.6 mm vias with 0.15 mm
        // clearance need about 1.559 mm; the earlier placement left 0.65 mm.
        assert!(mcu.min.y - bounds("R3").max.y >= 1.559);
        assert!(mcu.min.y - bounds("R4").max.y >= 1.559);
        assert!(bounds("C22").min.y - mcu.max.y >= 1.559);
        assert!(board.tracks.is_empty() && board.vias.is_empty());
    }

    #[test]
    fn test_framework_clock_boot_and_led_circuits_are_connected() {
        let board=create_framework_card_board();
        let report=crate::pcb::validate_board(&board,&crate::pcb::prototype_rules());
        assert!(!report.software_checks_pass);
        assert!(report.electrical_issues.is_empty(), "{:?}", report.electrical_issues);
        let net = |reference: &str, pin: &str| board.find_footprint(reference).unwrap().pads.iter()
            .find(|p| p.pin_number==pin).unwrap().net.unwrap();
        assert_ne!(net("U1", "21"), net("Y1", "3"));
        assert_eq!(net("R5", "1"), net("U1", "21"));
        assert_eq!(net("R5", "2"), net("Y1", "3"));
        assert_eq!(net("C4", "1"), net("Y1", "1"));
        assert_eq!(net("C5", "1"), net("Y1", "3"));
        assert_ne!(net("SW1", "2"), net("U2", "1"));
        assert_eq!(net("R6", "1"), net("U2", "1"));
        assert_eq!(net("R6", "2"), net("SW1", "2"));
        assert_eq!(net("LED1", "1"), net("U1", "57"));
        assert_ne!(net("LED1", "2"), net("U1", "18"));
        assert_eq!(net("R7", "1"), net("U1", "18"));
        assert_eq!(net("R7", "2"), net("LED1", "2"));
        assert!(!report.drc.unrouted_nets.is_empty());
    }
}
