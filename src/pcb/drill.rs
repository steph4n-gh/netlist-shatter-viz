//! Excellon NC Drill File Exporter.
//!
//! Generates CNC drilling toolpaths for PCB manufacturing in standard Excellon format:
//! - M48 header with metric millimeter format (`METRIC`).
//! - Deterministic tool definition table (`T01C...`) sorted by tool diameter.
//! - Tool selection (`T01`, `T02`, ...) and coordinate hits (`X...Y...`).
//! - Terminating `M30` directive.
//! - Extracts and groups all plated through-hole (THT) component pads and vias.

use std::collections::BTreeMap;
use crate::pcb::core::{PadType, PcbBoard, Point};

/// Single drill hole hit location with its associated drill diameter.
#[derive(Clone, Copy, Debug, PartialEq)]
struct DrillHole {
    pub diameter_nm: i64,
    pub at: Point,
}

/// Exports all drill hits (vias and through-hole component pads) into an Excellon NC drill file string.
pub fn export_excellon_drill(board: &PcbBoard) -> String {
    let mut holes: Vec<DrillHole> = Vec::new();

    // 1. Collect all via drill holes
    for via in &board.vias {
        if via.drill > 0.0 {
            let dia_nm = (via.drill * 1_000_000.0).round() as i64;
            holes.push(DrillHole {
                diameter_nm: dia_nm,
                at: via.at,
            });
        }
    }

    // 2. Collect all through-hole footprint pad drill holes
    for fp in &board.footprints {
        for pad in &fp.pads {
            if let PadType::ThroughHole { drill, .. } = pad.pad_type {
                if drill > 0.0 {
                    let dia_nm = (drill * 1_000_000.0).round() as i64;
                    let at = fp.pad_world_position(pad);
                    holes.push(DrillHole {
                        diameter_nm: dia_nm,
                        at,
                    });
                }
            }
        }
    }

    // 3. Group holes by tool diameter using BTreeMap for deterministic ascending order
    let mut tool_groups: BTreeMap<i64, Vec<Point>> = BTreeMap::new();
    for mut hole in holes {
        hole.at.x = (hole.at.x * 1000.0).round() / 1000.0;
        hole.at.y = ((board.height - hole.at.y) * 1000.0).round() / 1000.0;
        tool_groups.entry(hole.diameter_nm).or_default().push(hole.at);
    }

    // 4. Build Excellon drill file string
    let mut out = String::with_capacity(4096);

    // Header
    out.push_str("M48\n");
    out.push_str("; DRILL file {Netlist Shatter Excellon Exporter}\n");
    out.push_str("; FORMAT={-:-/ absolute / metric / decimal}\n");
    out.push_str("METRIC\n");

    // Tool Table Definition: T01C0.400, T02C1.000, ...
    for (tool_idx, (&dia_nm, _)) in tool_groups.iter().enumerate() {
        let tool_num = tool_idx + 1;
        let dia_mm = dia_nm as f64 / 1_000_000.0;
        out.push_str(&format!("T{:02}C{:.3}\n", tool_num, dia_mm));
    }

    out.push_str("%\n");
    out.push_str("G90\n");
    out.push_str("G05\n");

    // Tool Hits Body
    for (tool_idx, (_, hits)) in tool_groups.iter().enumerate() {
        let tool_num = tool_idx + 1;
        out.push_str(&format!("T{:02}\n", tool_num));

        // Sort hits deterministically (primary Y, secondary X) for efficient CNC toolpaths
        let mut sorted_hits = hits.clone();
        sorted_hits.sort_by(|a, b| {
            a.y.partial_cmp(&b.y)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| a.x.partial_cmp(&b.x).unwrap_or(std::cmp::Ordering::Equal))
        });

        // Deduplicate coincident hits for the same tool
        sorted_hits.dedup_by(|a, b| (a.x - b.x).abs() < 1e-4 && (a.y - b.y).abs() < 1e-4);

        for pt in sorted_hits {
            let norm_x = if pt.x.abs() < 1e-4 || (pt.x < 0.0 && pt.x > -0.0005) { 0.0 } else { pt.x };
            let norm_y = if pt.y.abs() < 1e-4 || (pt.y < 0.0 && pt.y > -0.0005) { 0.0 } else { pt.y };
            let x_str = format!("{:.3}", norm_x);
            let y_str = format!("{:.3}", norm_y);
            let x_str = if x_str == "-0.000" { "0.000" } else { &x_str };
            let y_str = if y_str == "-0.000" { "0.000" } else { &y_str };
            out.push_str(&format!("X{}Y{}\n", x_str, y_str));
        }
    }

    // End of Program Directive
    out.push_str("M30\n");

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pcb::core::{Point, Via};
    use crate::pcb::footprint::header_1x15;

    #[test]
    fn test_drill_export() {
        let mut board = PcbBoard::new(60.0, 35.0);
        let j1 = header_1x15("J1", "CONN_01x15").at_position(Point::new(10.0, 5.0));
        board.add_footprint(j1);
        board.add_via(Via::new(Point::new(25.0, 20.0), 0.8, 0.4, 1));

        let drl = export_excellon_drill(&board);
        assert!(drl.starts_with("M48"));
        assert!(drl.contains("METRIC"));
        assert!(drl.contains("T01C0.400"));
        assert!(drl.contains("T02C1.000"));
        assert!(drl.contains("X25.000Y15.000"));
        assert!(drl.trim_end().ends_with("M30"));

        let hit_count = drl
            .lines()
            .filter(|line| line.starts_with('X') && line.contains('Y'))
            .count();
        assert_eq!(hit_count, 16);
    }

    #[test]
    fn test_empty_drill_export() {
        let board = PcbBoard::new(50.0, 50.0);
        let drl = export_excellon_drill(&board);
        assert!(drl.starts_with("M48"));
        assert!(drl.trim_end().ends_with("M30"));
    }

    #[test]
    fn test_drill_deduplication() {
        let mut board = PcbBoard::new(50.0, 50.0);
        // Duplicate vias at same coordinate
        board.add_via(Via::new(Point::new(10.0, 10.0), 0.8, 0.4, 1));
        board.add_via(Via::new(Point::new(10.0, 10.0), 0.8, 0.4, 1));
        board.add_via(Via::new(Point::new(10.00001, 10.00001), 0.8, 0.4, 1));
        // Distinct via
        board.add_via(Via::new(Point::new(20.0, 20.0), 0.8, 0.4, 1));

        let drl = export_excellon_drill(&board);
        let hit_count = drl
            .lines()
            .filter(|line| line.starts_with('X') && line.contains('Y'))
            .count();
        assert_eq!(hit_count, 2, "Coincident drill hits must be deduplicated to a single hit");
        assert_eq!(drl.matches("X10.000Y40.000").count(), 1);
        assert_eq!(drl.matches("X20.000Y30.000").count(), 1);
    }

    #[test]
    fn test_drill_negative_zero_normalization() {
        let mut board = PcbBoard::new(50.0, 50.0);
        board.add_via(Via::new(Point::new(-0.0, 0.0), 0.8, 0.4, 1));
        board.add_via(Via::new(Point::new(0.0, -0.0), 0.8, 0.4, 1));
        board.add_via(Via::new(Point::new(-0.0002, 5.0), 0.8, 0.4, 1));
        board.add_via(Via::new(Point::new(5.0, -0.0004), 0.8, 0.4, 1));

        let drl = export_excellon_drill(&board);
        assert!(!drl.contains("-0.000"), "Drill file must never contain -0.000");
        assert!(drl.contains("X0.000Y50.000"));
        assert!(drl.contains("X0.000Y45.000"));
        assert!(drl.contains("X5.000Y50.000"));
    }
}

