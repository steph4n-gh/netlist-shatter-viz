//! M64 Cart Pro engineering draft.
//! See docs/DESIGN_REVIEW.md and generated report.json for known circuit blockers.
//! Routing success is not proof of electrical correctness or manufacturing qualification.

use crate::pcb::assembly::{BomEntry, CplEntry};
use crate::pcb::core::{NetId, PcbBoard, Point};
use crate::pcb::footprint::{
    capacitor_0805, crystal_smd_3225, hirose_fx8_50p, inductor_0805, led_0805, microsd_socket,
    qfn88, resistor_0805, soic8, sot23_5,
};
use crate::pcb::router::{route_board, RouterConfig, RoutingError};

/// Helper to assign a net to a pad identified by pin number or pin name.
#[inline]
fn assign_pad(board: &mut PcbBoard, comp_ref: &str, pin_id: &str, net: NetId) {
    let net_name=board.get_net_name(net).expect("Unknown net").to_owned();
    board.assign_pad_net(comp_ref,pin_id,&net_name).expect("Invalid board netlist");
}

/// Legacy custom cart concept; NOT the published ModRetro cartridge.
/// The official circuit is recovered in reference/m64-cartridge/.
/// This retained draft uses different parts and incomplete connections and must
/// not be used as a manufacturing source or an M64 optimization benchmark.
pub fn create_m64_cart_pro_board() -> PcbBoard {
    let mut board = PcbBoard::new(65.0, 45.0);

    // ========================================================================
    // 1. Physical Footprint Placement
    // ========================================================================

    // U1: Gowin GW1N-9 FPGA (QFN-88, 10x10 mm body, 0.4 mm pitch) centered at X=32.0, Y=21.0
    let u1 = qfn88("U1", "GW1N-UV9QN88").at_position(Point::new(32.0, 21.0));
    board.add_footprint(u1);

    // J1: Hirose FX8-50P-SV 50-pin Board-to-Board Mezzanine Receptacle at X=32.0, Y=40.0
    let j1 = hirose_fx8_50p("J1", "FX8-50P-SV").at_position(Point::new(32.0, 40.0));
    board.add_footprint(j1);

    // J2: Push-Push MicroSD Card Socket along left edge at X=10.0, Y=19.0 (card slot facing left)
    let j2 = microsd_socket("J2", "MicroSD").at_position(Point::new(10.0, 19.0));
    board.add_footprint(j2);

    // U2: Winbond W25Q128 16MB QSPI NOR Flash (SOIC-8) at X=48.0, Y=14.0
    let u2 = soic8("U2", "W25Q128JVS").at_position(Point::new(48.0, 14.0));
    board.add_footprint(u2);

    // U3: Fujitsu MB85RS256B 256Kbit SPI Non-Volatile FRAM (SOIC-8) at X=48.0, Y=27.0
    let u3 = soic8("U3", "MB85RS256B").at_position(Point::new(48.0, 27.0));
    board.add_footprint(u3);

    // Y1: 50.000 MHz Oscillator (SMD 3225 4-pin) at X=32.15, Y=8.5 (rotated 90° for direct pin 75 alignment)
    let y1 = crystal_smd_3225("Y1", "50MHz")
        .at_position(Point::new(32.15, 8.5))
        .with_rotation(90.0);
    board.add_footprint(y1);

    // U4: Diodes Inc. AP3428 1.2V Synchronous Buck Converter (SOT-23-5) at X=15.0, Y=34.0
    let u4 = sot23_5("U4", "AP3428-1.2V").at_position(Point::new(15.0, 34.0));
    board.add_footprint(u4);

    // L1: 2.2uH 0805 Power Inductor for AP3428 buck at X=19.5, Y=34.0
    let l1 = inductor_0805("L1", "2.2uH").at_position(Point::new(19.5, 34.0));
    board.add_footprint(l1);

    // C1: 10uF 0805 Input Decoupling Capacitor at X=10.0, Y=34.0
    let c1 = capacitor_0805("C1", "10uF").at_position(Point::new(10.0, 34.0));
    board.add_footprint(c1);

    // C2: 10uF 0805 1.2V Output Filter Capacitor at X=24.5, Y=34.0
    let c2 = capacitor_0805("C2", "10uF").at_position(Point::new(24.5, 34.0));
    board.add_footprint(c2);

    // C3: 0.1uF 0805 FPGA 1.2V Core Decoupling at X=23.0, Y=21.0
    let c3 = capacitor_0805("C3", "0.1uF").at_position(Point::new(23.0, 21.0));
    board.add_footprint(c3);

    // C4: 0.1uF 0805 Flash 3.3V Decoupling at X=48.0, Y=8.5
    let c4 = capacitor_0805("C4", "0.1uF").at_position(Point::new(48.0, 8.5));
    board.add_footprint(c4);

    // LED1: 0805 Status / FPGA Config Done LED at X=58.0, Y=14.0
    let led1 = led_0805("LED1", "DONE").at_position(Point::new(58.0, 14.0));
    board.add_footprint(led1);

    // R1: 1.0k 0805 LED Current Limiting Resistor at X=54.0, Y=14.0
    let r1 = resistor_0805("R1", "1.0k").at_position(Point::new(54.0, 14.0));
    board.add_footprint(r1);

    // ========================================================================
    // 2. Net Registration
    // ========================================================================

    let net_gnd = board.get_or_create_net("GND");
    let net_3v3 = board.get_or_create_net("3V3_SYS");
    let net_1v2 = board.get_or_create_net("1V2_CORE");
    let net_sw_node = board.get_or_create_net("BUCK_SW");
    let net_clk50 = board.get_or_create_net("SYS_CLK_50M");
    let net_done = board.get_or_create_net("FPGA_DONE");
    let net_led_k = board.get_or_create_net("LED_CATHODE");

    // Flash QSPI
    let net_flash_clk = board.get_or_create_net("FLASH_CLK");
    let net_flash_cs = board.get_or_create_net("FLASH_CS_N");
    let net_flash_d0 = board.get_or_create_net("FLASH_IO0");
    let net_flash_d1 = board.get_or_create_net("FLASH_IO1");

    // FRAM SPI
    let net_fram_clk = board.get_or_create_net("FRAM_CLK");
    let net_fram_cs = board.get_or_create_net("FRAM_CS_N");
    let net_fram_si = board.get_or_create_net("FRAM_SI");
    let net_fram_so = board.get_or_create_net("FRAM_SO");

    // MicroSD
    let net_sd_clk = board.get_or_create_net("SD_CLK");
    let net_sd_cmd = board.get_or_create_net("SD_CMD");
    let net_sd_dat0 = board.get_or_create_net("SD_DAT0");
    let net_sd_cd = board.get_or_create_net("SD_CD");

    // M64 / N64 Bus Signals on Hirose J1
    let net_n64_ad0 = board.get_or_create_net("N64_AD0");
    let net_n64_ad1 = board.get_or_create_net("N64_AD1");
    let net_n64_ad2 = board.get_or_create_net("N64_AD2");
    let net_n64_ad3 = board.get_or_create_net("N64_AD3");
    let net_n64_ale_l = board.get_or_create_net("N64_ALE_L");
    let net_n64_read = board.get_or_create_net("N64_READ_N");

    // ========================================================================
    // 3. Netlist Connectivity
    // ========================================================================

    // Power: Ground
    assign_pad(&mut board, "U1", "89", net_gnd); // FPGA EPAD
    assign_pad(&mut board, "J1", "SH1", net_gnd);
    assign_pad(&mut board, "J1", "SH2", net_gnd);
    assign_pad(&mut board, "J1", "1", net_gnd);
    assign_pad(&mut board, "J1", "25", net_gnd);
    assign_pad(&mut board, "J2", "6", net_gnd); // MicroSD VSS
    assign_pad(&mut board, "J2", "SH1", net_gnd);
    assign_pad(&mut board, "J2", "SH2", net_gnd);
    assign_pad(&mut board, "U2", "4", net_gnd); // Flash GND
    assign_pad(&mut board, "U3", "4", net_gnd); // FRAM GND
    assign_pad(&mut board, "U4", "2", net_gnd); // AP3428 GND
    assign_pad(&mut board, "C1", "2", net_gnd);
    assign_pad(&mut board, "C2", "2", net_gnd);
    assign_pad(&mut board, "C3", "2", net_gnd);
    assign_pad(&mut board, "C4", "2", net_gnd);
    assign_pad(&mut board, "Y1", "2", net_gnd);
    assign_pad(&mut board, "LED1", "2", net_gnd);

    // Power: 3.3V System Distribution (from Hirose J1 Pin 2)
    assign_pad(&mut board, "J1", "2", net_3v3);
    assign_pad(&mut board, "U4", "1", net_3v3); // Buck VIN
    assign_pad(&mut board, "U4", "3", net_3v3); // Buck EN
    assign_pad(&mut board, "C1", "1", net_3v3);
    assign_pad(&mut board, "U1", "1", net_3v3); // FPGA VCCO
    assign_pad(&mut board, "U2", "8", net_3v3); // Flash VCC
    assign_pad(&mut board, "U2", "3", net_3v3); // Flash WP# high
    assign_pad(&mut board, "U2", "7", net_3v3); // Flash HOLD# high
    assign_pad(&mut board, "U3", "8", net_3v3); // FRAM VDD
    assign_pad(&mut board, "U3", "3", net_3v3); // FRAM WP# high
    assign_pad(&mut board, "U3", "7", net_3v3); // FRAM HOLD# high
    assign_pad(&mut board, "C4", "1", net_3v3);
    assign_pad(&mut board, "J2", "4", net_3v3); // MicroSD VDD
    assign_pad(&mut board, "Y1", "4", net_3v3); // Oscillator VDD

    // Power: 1.2V Core Generation (AP3428 Synchronous Buck)
    assign_pad(&mut board, "U4", "5", net_sw_node); // LX / SW pin
    assign_pad(&mut board, "L1", "1", net_sw_node);
    assign_pad(&mut board, "L1", "2", net_1v2);
    assign_pad(&mut board, "U4", "4", net_1v2); // FB feedback pin
    assign_pad(&mut board, "C2", "1", net_1v2);
    assign_pad(&mut board, "C3", "1", net_1v2);
    assign_pad(&mut board, "U1", "22", net_1v2); // FPGA VCC core

    // Clock System (50 MHz)
    assign_pad(&mut board, "Y1", "3", net_clk50); // Oscillator OUT
    assign_pad(&mut board, "U1", "75", net_clk50); // FPGA GCLK top pin

    // Status / Config DONE LED
    assign_pad(&mut board, "U1", "55", net_done); // FPGA DONE pin
    assign_pad(&mut board, "R1", "1", net_done);
    assign_pad(&mut board, "R1", "2", net_led_k);
    assign_pad(&mut board, "LED1", "1", net_led_k);

    // QSPI Flash Interface (Winbond W25Q128)
    assign_pad(&mut board, "U1", "58", net_flash_clk);
    assign_pad(&mut board, "U2", "6", net_flash_clk);
    assign_pad(&mut board, "U1", "60", net_flash_cs);
    assign_pad(&mut board, "U2", "1", net_flash_cs);
    assign_pad(&mut board, "U1", "61", net_flash_d0);
    assign_pad(&mut board, "U2", "5", net_flash_d0);
    assign_pad(&mut board, "U1", "62", net_flash_d1);
    assign_pad(&mut board, "U2", "2", net_flash_d1);

    // SPI FRAM Interface (Fujitsu MB85RS256B)
    assign_pad(&mut board, "U1", "48", net_fram_clk);
    assign_pad(&mut board, "U3", "6", net_fram_clk);
    assign_pad(&mut board, "U1", "49", net_fram_cs);
    assign_pad(&mut board, "U3", "1", net_fram_cs);
    assign_pad(&mut board, "U1", "50", net_fram_si);
    assign_pad(&mut board, "U3", "5", net_fram_si);
    assign_pad(&mut board, "U1", "51", net_fram_so);
    assign_pad(&mut board, "U3", "2", net_fram_so);

    // MicroSD Card Interface (SPI / SD Mode)
    assign_pad(&mut board, "U1", "11", net_sd_clk);
    assign_pad(&mut board, "J2", "5", net_sd_clk);
    assign_pad(&mut board, "U1", "12", net_sd_cmd);
    assign_pad(&mut board, "J2", "3", net_sd_cmd);
    assign_pad(&mut board, "U1", "13", net_sd_dat0);
    assign_pad(&mut board, "J2", "7", net_sd_dat0);
    assign_pad(&mut board, "U1", "14", net_sd_cd);
    // This socket has no separate card-detect pad. SD_CD remains a documented draft blocker.

    // M64 Console Bus (Hirose FX8-50P J1 -> FPGA U1)
    assign_pad(&mut board, "J1", "10", net_n64_ad0);
    assign_pad(&mut board, "U1", "30", net_n64_ad0);

    assign_pad(&mut board, "J1", "12", net_n64_ad1);
    assign_pad(&mut board, "U1", "32", net_n64_ad1);

    assign_pad(&mut board, "J1", "14", net_n64_ad2);
    assign_pad(&mut board, "U1", "34", net_n64_ad2);

    assign_pad(&mut board, "J1", "16", net_n64_ad3);
    assign_pad(&mut board, "U1", "36", net_n64_ad3);

    assign_pad(&mut board, "J1", "20", net_n64_ale_l);
    assign_pad(&mut board, "U1", "40", net_n64_ale_l);

    assign_pad(&mut board, "J1", "22", net_n64_read);
    assign_pad(&mut board, "U1", "42", net_n64_read);

    board
}

/// Routes the legacy custom cart concept; this is not the published ModRetro circuit.
pub fn solve_m64_cart_pro_board(config: &RouterConfig) -> Result<PcbBoard, RoutingError> {
    let mut board = create_m64_cart_pro_board();
    route_board(&mut board, config)?;
    Ok(board)
}

/// Generates the Bill of Materials (BOM) for the legacy custom cart concept.
pub fn generate_m64_cart_pro_bom() -> Vec<BomEntry> {
    vec![
        BomEntry::new(1, "U1", "GW1N-UV9QN88", "QFN-88-1EP_10x10mm_P0.4mm", 1, "Gowin LittleBee GW1N-9 8640 LUT Low-Power FPGA", "Gowin Semiconductor", "GW1N-UV9QN88C6/I5"),
        BomEntry::new(2, "U2", "W25Q128JVS", "SOIC-8_3.9x4.9mm_P1.27mm", 1, "16MB (128M-bit) High-Speed QSPI Serial NOR Flash Memory", "Winbond", "W25Q128JVSSIQ"),
        BomEntry::new(3, "U3", "MB85RS256B", "SOIC-8_3.9x4.9mm_P1.27mm", 1, "256Kbit (32K x 8) High-Reliability SPI Non-Volatile FRAM 33MHz", "Fujitsu", "MB85RS256BPNF-G-JNE1"),
        BomEntry::new(4, "U4", "AP3428WT-7", "SOT-23-5", 1, "1A Synchronous DC-DC Step-Down Buck Converter 1.2V Core Rail", "Diodes Inc.", "AP3428WT-7"),
        BomEntry::new(5, "J1", "FX8-50P-SV", "Hirose_FX8-50P-SV_2x25_P0.60mm_Vertical", 1, "50-Pin 0.6mm Pitch SMT Mezzanine Board-to-Board Receptacle", "Hirose Electric", "FX8-50P-SV(21)"),
        BomEntry::new(6, "J2", "MicroSD Socket", "MicroSD_Socket_PushPush", 1, "Push-Push MicroSD Memory Card Receptacle SMT 9-Pin", "Korean Hroparts", "TF-01A"),
        BomEntry::new(7, "Y1", "50.000MHz", "Crystal_SMD_3225-4Pin", 1, "Ultra-Low Jitter Clock Oscillator 50.000MHz 3.3V SMD 3225", "ECS Inc.", "ECS-3225MV-500-BN-TR"),
        BomEntry::new(8, "L1", "2.2uH", "L_0805_2012Metric", 1, "Multilayer Power Inductor 2.2uH 1.2A 0805", "Murata", "LQM21PN2R2MC0D"),
        BomEntry::new(9, "LED1", "Green LED", "LED_0805_2012Metric", 1, "Surface Mount LED 0805 Green 20mA (FPGA Config DONE)", "Kingbright", "APT2012SGC"),
        BomEntry::new(10, "R1", "1.0k", "R_0805_2012Metric", 1, "Thick Film Resistor 1.0k Ohm 1% 1/8W 0805 (LED Current Limit)", "Yageo", "RC0805FR-071KL"),
        BomEntry::new(11, "C1, C2", "10uF", "C_0805_2012Metric", 2, "Ceramic Capacitor 10uF 16V X7R 0805 (Buck Input/Output Filter)", "Murata", "GRM21BR71C106KE15L"),
        BomEntry::new(12, "C3, C4", "0.1uF", "C_0805_2012Metric", 2, "Ceramic Capacitor 0.1uF 25V X7R 0805 (Core / Flash High-Freq Decoupling)", "Murata", "GRM21BR71E104KA01L"),
    ]
}

/// Generates the Pick-and-Place (CPL / Centroid) list for the legacy custom cart concept.
pub fn generate_m64_cart_pro_cpl(board: &PcbBoard) -> Vec<CplEntry> {
    board
        .footprints
        .iter()
        .map(|fp| {
            CplEntry::new(
                &fp.reference,
                fp.position.x,
                fp.position.y,
                "Top",
                fp.rotation,
            )
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pcb::core::compute_ratsnest;
    use crate::pcb::drc::run_drc;
    use crate::pcb::router::{route_board, RouterConfig};

    #[test]
    fn test_m64_cart_pro_geometry_and_counts() {
        let board = create_m64_cart_pro_board();
        assert_eq!(board.width, 65.0);
        assert_eq!(board.height, 45.0);
        assert_eq!(board.footprints.len(), 14);

        let total_pads: usize = board.footprints.iter().map(|fp| fp.pads.len()).sum();
        assert_eq!(total_pads, 192);
    }

    #[test]
    fn test_m64_cart_pro_netlist_and_ratsnest() {
        let board = create_m64_cart_pro_board();
        let ratsnest = compute_ratsnest(&board);
        assert!(!ratsnest.is_empty(), "Ratsnest must contain lines before routing");

        assert!(board.net_names.values().any(|n| n == "GND"));
        assert!(board.net_names.values().any(|n| n == "3V3_SYS"));
        assert!(board.net_names.values().any(|n| n == "1V2_CORE"));
        assert!(board.net_names.values().any(|n| n == "SYS_CLK_50M"));
        assert!(board.net_names.values().any(|n| n == "FLASH_CLK"));
        assert!(board.net_names.values().any(|n| n == "FRAM_CLK"));
        assert!(board.net_names.values().any(|n| n == "SD_CLK"));
        assert!(board.net_names.values().any(|n| n == "N64_AD0"));
    }

    #[test]
    fn test_m64_cart_pro_autorouting_and_drc() {
        let mut board = create_m64_cart_pro_board();
        let config = RouterConfig {
            grid_resolution: 0.20,
            default_trace_width: 0.20,
            via_diameter: 0.60,
            via_drill: 0.30,
            clearance: 0.15,
        };

        let result = route_board(&mut board, &config);
        if let Err(e) = &result {
            eprintln!("M64 Cart Pro Routing failed with: {:?}", e);
            for (id, name) in &board.net_names {
                eprintln!("Net {}: {}", id, name);
            }
        }
        assert!(result.is_ok(), "M64 Cart Pro autorouting must succeed: {:?}", result);

        let drc = run_drc(&board, &config);
        for sc in &drc.short_circuits {
            eprintln!("DRC Short: {:?}", sc);
        }
        for cv in &drc.clearance_violations {
            eprintln!("DRC Clearance: {:?}", cv);
        }
        assert_eq!(drc.short_circuits.len(), 0, "Zero short circuits");
        assert_eq!(drc.clearance_violations.len(), 0, "Zero clearance violations");
        assert_eq!(drc.annular_ring_violations.len(), 0, "Zero annular ring violations");
        assert_eq!(drc.edge_keepout_violations.len(), 0, "Zero edge keepout violations");
        assert_eq!(drc.unrouted_nets.len(), 0, "All nets routed");
    }
}
