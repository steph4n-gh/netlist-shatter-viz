//! KiCad 7/8 S-Expression PCB Exporter (`.kicad_pcb`).
//!
//! Generates valid, parseable KiCad S-expression documents with strictly balanced
//! parentheses, standard headers, net tables, placed footprints, copper tracks,
//! plated vias, and board outline Edge.Cuts graphical lines.

use crate::pcb::core::{Footprint, Layer, PadShape, PadType, PcbBoard, Point, Rect};

/// Printed outlines must clear every exposed pad, including neighboring parts.
/// Keep the complete component outline on F.Fab; clip only the printed strokes.
pub(crate) fn cleared_silkscreen(board: &PcbBoard, fp: &Footprint) -> Vec<(Point, Point)> {
    let mut masks: Vec<_> = board.footprints.iter().flat_map(|owner| {
        owner.pads.iter().filter_map(move |pad| {
            if matches!(pad.pad_type, PadType::Smd { layer: Layer::BCu }) {
                return None;
            }
            let mut bounds = owner.pad_world_bbox(pad);
            // 0.05 mm mask expansion + 0.15 mm silk clearance + 0.06 mm
            // half-stroke; the small extra guard covers coordinate rounding.
            bounds.min.x -= 0.261;
            bounds.min.y -= 0.261;
            bounds.max.x += 0.261;
            bounds.max.y += 0.261;
            Some(bounds)
        })
    }).collect();
    // Adjacent components can share a body edge. Print it once rather than
    // producing duplicate or crossing strokes; retain both bodies on F.Fab.
    for previous in &board.footprints {
        if previous.reference == fp.reference { break; }
        for (p, q) in previous.silkscreen_world_lines() {
            let mut bounds = Rect::new(p, q);
            bounds.min.x -= 0.271;
            bounds.min.y -= 0.271;
            bounds.max.x += 0.271;
            bounds.max.y += 0.271;
            masks.push(bounds);
        }
    }
    let mut result = Vec::new();
    for (start, end) in fp.silkscreen_world_lines() {
        let delta = end - start;
        let mut intervals = vec![(0.0_f64, 1.0_f64)];
        for bounds in &masks {
            let mut lo = 0.0_f64;
            let mut hi = 1.0_f64;
            for (at, direction, min, max) in [
                (start.x, delta.x, bounds.min.x, bounds.max.x),
                (start.y, delta.y, bounds.min.y, bounds.max.y),
            ] {
                if direction.abs() < 1e-12 {
                    if at < min || at > max { lo = 1.0; hi = 0.0; break; }
                } else {
                    let a = (min - at) / direction;
                    let b = (max - at) / direction;
                    lo = lo.max(a.min(b));
                    hi = hi.min(a.max(b));
                }
            }
            if lo >= hi { continue; }
            intervals = intervals.into_iter().flat_map(|(a, b)| {
                let mut remaining = Vec::new();
                if b <= lo || a >= hi { remaining.push((a, b)); }
                else {
                    if a < lo { remaining.push((a, lo)); }
                    if b > hi { remaining.push((hi, b)); }
                }
                remaining
            }).collect();
        }
        for (a, b) in intervals {
            let p = start + delta * a;
            let q = start + delta * b;
            if p.distance_to(q) >= 0.15 { result.push((p, q)); }
        }
    }
    result
}

/// Formats a floating-point coordinate or dimension cleanly for KiCad S-expressions.
///
/// Trims trailing zeros and avoids exponential notation or precision artifacts.
#[inline]
fn format_coord(val: f64) -> String {
    if val.abs() < 1e-9 {
        return "0".to_string();
    }
    // Match KiCad's native nanometre coordinates so imported repair routes
    // retain their exact endpoints and via centers during manufacturing export.
    let rounded = (val * 1_000_000.0).round() / 1_000_000.0;
    let s = format!("{:.6}", rounded);
    let trimmed = s.trim_end_matches('0');
    let res = trimmed.trim_end_matches('.');
    if res.is_empty() || res == "-0" {
        "0".to_string()
    } else {
        res.to_string()
    }
}

/// Escapes strings for KiCad S-expression string literals.
fn escape_string(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

/// Exports a complete `PcbBoard` assembly into a valid KiCad 7/8 `.kicad_pcb` S-expression string.
pub fn export_kicad_pcb(board: &PcbBoard) -> String {
    let mut out = String::with_capacity(8192);

    // 1. Root Header & Metadata
    out.push_str("(kicad_pcb (version 20240108) (generator netlist_shatter_viz)\n\n");
    out.push_str("  (general\n");
    out.push_str(&format!("    (thickness {})\n", format_coord(board.thickness)));
    out.push_str("  )\n\n");
    out.push_str("  (paper \"A4\")\n\n");

    // 2. Layer Table Definition
    out.push_str("  (layers\n");
    out.push_str("    (0 \"F.Cu\" signal)\n");
    out.push_str("    (31 \"B.Cu\" signal)\n");
    out.push_str("    (34 \"B.Paste\" user)\n    (35 \"F.Paste\" user)\n");
    out.push_str("    (36 \"B.SilkS\" user \"B.Silkscreen\")\n");
    out.push_str("    (37 \"F.SilkS\" user \"F.Silkscreen\")\n");
    out.push_str("    (38 \"B.Mask\" user)\n");
    out.push_str("    (39 \"F.Mask\" user)\n");
    out.push_str("    (44 \"Edge.Cuts\" user)\n    (49 \"F.Fab\" user)\n");
    out.push_str("  )\n\n");

    out.push_str("  (setup (pad_to_mask_clearance 0))\n");

    // 3. Netlist Table
    // KiCad mandates net 0 as (net 0 "")
    out.push_str("  (net 0 \"\")\n");

    // Deterministically sort nets by NetId
    let mut sorted_nets: Vec<(u32, &String)> = board
        .net_names
        .iter()
        .map(|(&id, name)| (id, name))
        .collect();
    sorted_nets.sort_by_key(|&(id, _)| id);

    for (id, name) in sorted_nets {
        if id != 0 {
            out.push_str(&format!("  (net {} \"{}\")\n", id, escape_string(name)));
        }
    }
    out.push('\n');

    // 4. Placed Footprints
    for fp in &board.footprints {
        let package_esc = escape_string(&fp.package);
        let ref_esc = escape_string(&fp.reference);
        let val_esc = escape_string(&fp.value);

        let at_str = if fp.rotation.abs() > 1e-6 {
            format!(
                "(at {} {} {})",
                format_coord(fp.position.x),
                format_coord(fp.position.y),
                format_coord(-fp.rotation)
            )
        } else {
            format!(
                "(at {} {})",
                format_coord(fp.position.x),
                format_coord(fp.position.y)
            )
        };

        out.push_str(&format!(
            "  (footprint \"{}\" (layer \"F.Cu\")\n    {}\n",
            package_esc, at_str
        ));

        // Preserve the model's exact population directive and known mechanical features.
        let mechanical = matches!(fp.package.as_str(), "FIDUCIAL_1MM" | "MOUNTINGHOLE_2.5_PLATED");
        if fp.value == "DNP" || mechanical {
            out.push_str("    (attr");
            if mechanical { out.push_str(" exclude_from_pos_files exclude_from_bom"); }
            if fp.value == "DNP" { out.push_str(" dnp"); }
            out.push_str(")\n");
        }

        // Standard KiCad reference & value properties
        out.push_str(&format!(
            "    (property \"Reference\" \"{}\" (at 0 -2 0) (layer \"F.Fab\") (effects (font (size 1 1) (thickness 0.15))))\n",
            ref_esc
        ));
        out.push_str(&format!(
            "    (property \"Value\" \"{}\" (at 0 2 0) (layer \"F.Fab\") (effects (font (size 1 1) (thickness 0.15))))\n",
            val_esc
        ));

        // Complete body geometry remains available to assembly on F.Fab.
        for &(p1, p2) in &fp.silkscreen_lines {
            out.push_str(&format!(
                "    (fp_line (start {} {}) (end {} {}) (layer \"F.Fab\") (width 0.12))\n",
                format_coord(p1.x),
                format_coord(p1.y),
                format_coord(p2.x),
                format_coord(p2.y)
            ));
        }
        for (p, q) in cleared_silkscreen(board, fp) {
            let p = (p - fp.position).rotate(-fp.rotation);
            let q = (q - fp.position).rotate(-fp.rotation);
            out.push_str(&format!(
                "    (fp_line (start {} {}) (end {} {}) (layer \"F.SilkS\") (width 0.12))\n",
                format_coord(p.x), format_coord(p.y), format_coord(q.x), format_coord(q.y)
            ));
        }

        // Footprint pads
        for pad in &fp.pads {
            let pin_esc = escape_string(&pad.pin_number);
            let shape_str = pad.shape.kicad_shape_name();
            let rel_x = format_coord(pad.relative_pos.x);
            let rel_y = format!("{} {}", format_coord(pad.relative_pos.y), format_coord(-fp.rotation));

            let (size_w, size_h) = match pad.shape {
                PadShape::Rect { width, height } | PadShape::Obround { width, height } => {
                    let w = if width > 0.0 {
                        width
                    } else if let PadType::ThroughHole { outer_diameter, .. } = pad.pad_type {
                        outer_diameter
                    } else {
                        1.0
                    };
                    let h = if height > 0.0 {
                        height
                    } else if let PadType::ThroughHole { outer_diameter, .. } = pad.pad_type {
                        outer_diameter
                    } else {
                        1.0
                    };
                    (format_coord(w), format_coord(h))
                }
                PadShape::Circle { diameter } => {
                    let d = if diameter > 0.0 {
                        diameter
                    } else if let PadType::ThroughHole { outer_diameter, .. } = pad.pad_type {
                        outer_diameter
                    } else {
                        1.0
                    };
                    (format_coord(d), format_coord(d))
                }
            };

            let net_str = match pad.net {
                Some(net_id) if net_id > 0 => {
                    let net_name = board.get_net_name(net_id).unwrap_or("");
                    format!(" (net {} \"{}\")", net_id, escape_string(net_name))
                }
                _ => String::new(),
            };

            match pad.pad_type {
                PadType::Smd { layer } => {
                    let layer_name = match layer {
                        Layer::BCu => "\"B.Cu\" \"B.Mask\" \"B.Paste\"",
                        _ => "\"F.Cu\" \"F.Mask\" \"F.Paste\"",
                    };
                    out.push_str(&format!(
                        "    (pad \"{}\" smd {} (at {} {}) (size {} {}) (layers {}){})\n",
                        pin_esc, shape_str, rel_x, rel_y, size_w, size_h, layer_name, net_str
                    ));
                }
                PadType::ThroughHole { drill, .. } => {
                    out.push_str(&format!(
                        "    (pad \"{}\" thru_hole {} (at {} {}) (size {} {}) (drill {}) (layers \"*.Cu\" \"*.Mask\"){})\n",
                        pin_esc,
                        shape_str,
                        rel_x,
                        rel_y,
                        size_w,
                        size_h,
                        format_coord(drill),
                        net_str
                    ));
                }
            }
        }

        out.push_str("  )\n\n");
    }

    // 5. Copper Tracks (FCu / BCu)
    for track in &board.tracks {
        let layer_name = track.layer.kicad_name();
        if !matches!(track.layer, Layer::FCu | Layer::BCu) {
            out.push_str(&format!("  (gr_line (start {} {}) (end {} {}) (stroke (width {}) (type default)) (layer \"{}\"))\n", format_coord(track.start.x),format_coord(track.start.y),format_coord(track.end.x),format_coord(track.end.y),format_coord(track.width),layer_name));
            continue;
        }
        out.push_str(&format!(
            "  (segment (start {} {}) (end {} {}) (width {}) (layer \"{}\") (net {}))\n",
            format_coord(track.start.x),
            format_coord(track.start.y),
            format_coord(track.end.x),
            format_coord(track.end.y),
            format_coord(track.width),
            layer_name,
            track.net
        ));
    }
    if !board.tracks.is_empty() {
        out.push('\n');
    }

    // 6. Plated Vias
    for via in &board.vias {
        out.push_str(&format!(
            "  (via (at {} {}) (size {}) (drill {}) (layers \"{}\" \"{}\") (net {}))\n",
            format_coord(via.at.x),
            format_coord(via.at.y),
            format_coord(via.size),
            format_coord(via.drill),
            via.layers.0.kicad_name(),
            via.layers.1.kicad_name(),
            via.net
        ));
    }
    if !board.vias.is_empty() {
        out.push('\n');
    }

    // 7. Board Boundary (Edge.Cuts)
    let edge_cuts = board.edge_cuts();
    for &(p1, p2) in &edge_cuts.segments {
        out.push_str(&format!(
            "  (gr_line (start {} {}) (end {} {}) (layer \"Edge.Cuts\") (width 0.15))\n",
            format_coord(p1.x),
            format_coord(p1.y),
            format_coord(p2.x),
            format_coord(p2.y)
        ));
    }

    // 8. Close Root Document S-expression
    out.push_str(")\n");

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pcb::core::{Point, TrackSegment, Via};
    use crate::pcb::footprint::{header_1x15, resistor_0805};

    fn assert_balanced(s: &str) {
        let mut depth = 0i32;
        for (idx, line) in s.lines().enumerate() {
            for c in line.chars() {
                if c == '(' {
                    depth += 1;
                } else if c == ')' {
                    depth -= 1;
                    assert!(depth >= 0, "Premature close at line {}: {}", idx + 1, line);
                }
            }
        }
        assert_eq!(depth, 0, "Unbalanced parentheses in S-expr");
    }

    #[test]
    fn test_sample_kicad_export() {
        let mut board = PcbBoard::new(60.0, 35.0);
        board.register_net(1, "GND");
        board.register_net(2, "+5V_USB");
        let r1 = resistor_0805("R1", "10k")
            .at_position(Point::new(15.0, 15.0))
            .with_rotation(90.0)
            .with_net_assigned("1", 2);
        board.add_footprint(r1);

        let j1 = header_1x15("J1", "CONN_01x15")
            .at_position(Point::new(10.0, 5.0))
            .with_net_assigned("1", 1);
        board.add_footprint(j1);

        board.add_track(TrackSegment::new(
            Point::new(15.0, 15.0),
            Point::new(25.0, 15.0),
            0.25,
            Layer::FCu,
            2,
        ));
        board.add_via(Via::new(Point::new(25.0, 15.0), 0.8, 0.4, 2));

        let out = export_kicad_pcb(&board);
        assert_balanced(&out);
        assert!(out.starts_with("(kicad_pcb"));
        assert!(out.contains("(net 0 \"\")"));
        assert!(out.contains("(net 1 \"GND\")"));
        assert!(out.contains("(net 2 \"+5V_USB\")"));
        assert!(out.contains("(width 0.25)"));
        assert!(out.contains("(size 0.8)"));
        assert!(out.contains("(drill 0.4)"));
        assert!(out.contains("(layer \"Edge.Cuts\")"));
    }

    #[test]
    fn test_exact_population_directive_and_mechanical_packages() {
        for (value, package, expected) in [
            ("DNP", "R_0805_link", "    (attr dnp)\n"),
            ("", "FIDUCIAL_1MM", "    (attr exclude_from_pos_files exclude_from_bom)\n"),
            ("MOUNTINGHOLE2.5", "MOUNTINGHOLE_2.5_PLATED", "    (attr exclude_from_pos_files exclude_from_bom)\n"),
            ("DNP", "FIDUCIAL_1MM", "    (attr exclude_from_pos_files exclude_from_bom dnp)\n"),
            ("dnp", "R_0805_link", ""),
            (" DNP", "R_0805_link", ""),
            ("DNP option", "R_0805_link", ""),
            ("10k", "R_DNP", ""),
            ("FIDUCIAL_1MM", "R_0805_link", ""),
            ("", "FIDUCIAL_1MM_OPTION", ""),
            ("", "MOUNTINGHOLE_2.5_PLATED_OPTION", ""),
        ] {
            let mut board = PcbBoard::new(20.0, 20.0);
            let mut fp = resistor_0805("R1", value).at_position(Point::new(5.0, 7.0)).with_rotation(90.0);
            fp.package = package.to_string();
            board.add_footprint(fp);
            let output = export_kicad_pcb(&board);
            let attrs: String = output.lines().filter(|line| line.trim_start().starts_with("(attr ")).map(|line| format!("{line}\n")).collect();
            assert_eq!(attrs, expected, "value={value:?}, package={package:?}");
            assert!(output.contains(&format!("(property \"Value\" \"{}\"", escape_string(value))));
            assert!(output.contains("(at 5 7 -90)"));
            assert_eq!(output.matches("(pad ").count(), 2);
            assert_balanced(&output);
        }
    }
}
