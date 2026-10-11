//! RS-274X Gerber Exporter.
//!
//! Generates standard RS-274X fabrication files for all standard PCB layers:
//! - Top Copper (`F_Cu.gbr`)
//! - Bottom Copper (`B_Cu.gbr`)
//! - Top Silkscreen (`F_SilkS.gbr`)
//! - Board Boundary Outline (`Edge_Cuts.gbr`)
//! - Top Solder Mask (`F_Mask.gbr`)
//! - Bottom Solder Mask (`B_Mask.gbr`)
//!
//! Adheres strictly to the RS-274X specification:
//! - Leading-zero suppressed, absolute coordinate scaling (%FSLAX46Y46*%, 10^6 nm integer units).
//! - Metric millimeter mode (%MOMM*%).
//! - Layer polarity dark (%LPD*%).
//! - Dynamic aperture dictionary (%ADD10C...%, %ADD...R...%, %ADD...O...%) indexed from D10.
//! - Linear draw (D01), positioning move (D02), and aperture flash (D03).
//! - Terminating M02* directive.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use crate::pcb::core::{Layer, PadShape, PadType, PcbBoard, Point};
use crate::pcb::drill::export_excellon_drill;

/// Canonical aperture definition with dimensions stored in integer nanometers (10^-6 mm).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Aperture {
    Circle { diameter_nm: i64 },
    Rect { width_nm: i64, height_nm: i64 },
    Obround { width_nm: i64, height_nm: i64 },
}

impl Aperture {
    pub fn circle(diameter_mm: f64) -> Self {
        let dia_nm = (diameter_mm * 1_000_000.0).round() as i64;
        Self::Circle {
            diameter_nm: dia_nm.max(1),
        }
    }

    pub fn rect(width_mm: f64, height_mm: f64, rotation_deg: f64) -> Self {
        let norm_rot = ((rotation_deg % 360.0) + 360.0) % 360.0;
        let is_transposed = (norm_rot - 90.0).abs() < 1e-9 || (norm_rot - 270.0).abs() < 1e-9;
        let (w, h) = if is_transposed {
            (height_mm, width_mm)
        } else {
            (width_mm, height_mm)
        };
        let w_nm = (w * 1_000_000.0).round() as i64;
        let h_nm = (h * 1_000_000.0).round() as i64;
        Self::Rect {
            width_nm: w_nm.max(1),
            height_nm: h_nm.max(1),
        }
    }

    pub fn obround(width_mm: f64, height_mm: f64, rotation_deg: f64) -> Self {
        let norm_rot = ((rotation_deg % 360.0) + 360.0) % 360.0;
        let is_transposed = (norm_rot - 90.0).abs() < 1e-9 || (norm_rot - 270.0).abs() < 1e-9;
        let (w, h) = if is_transposed {
            (height_mm, width_mm)
        } else {
            (width_mm, height_mm)
        };
        let w_nm = (w * 1_000_000.0).round() as i64;
        let h_nm = (h * 1_000_000.0).round() as i64;
        Self::Obround {
            width_nm: w_nm.max(1),
            height_nm: h_nm.max(1),
        }
    }

    pub fn from_pad_shape(shape: &PadShape, rotation_deg: f64) -> Self {
        match *shape {
            PadShape::Circle { diameter } => Self::circle(diameter),
            PadShape::Rect { width, height } => Self::rect(width, height, rotation_deg),
            PadShape::Obround { width, height } => Self::obround(width, height, rotation_deg),
        }
    }

    pub fn from_tht_pad(shape: &PadShape, outer_diameter: f64, rotation_deg: f64) -> Self {
        match *shape {
            PadShape::Circle { diameter } => {
                let dia = if diameter > 0.0 { diameter } else { outer_diameter };
                Self::circle(dia)
            }
            PadShape::Rect { width, height } => {
                let w = if width > 0.0 { width } else { outer_diameter };
                let h = if height > 0.0 { height } else { outer_diameter };
                Self::rect(w, h, rotation_deg)
            }
            PadShape::Obround { width, height } => {
                let w = if width > 0.0 { width } else { outer_diameter };
                let h = if height > 0.0 { height } else { outer_diameter };
                Self::obround(w, h, rotation_deg)
            }
        }
    }

    /// Formats the RS-274X %ADD...% directive for this aperture.
    pub fn format_add_directive(&self, code: u32) -> String {
        match *self {
            Self::Circle { diameter_nm } => {
                let dia_mm = diameter_nm as f64 / 1_000_000.0;
                format!("%ADD{}C,{:.4}*%\n", code, dia_mm)
            }
            Self::Rect {
                width_nm,
                height_nm,
            } => {
                let w_mm = width_nm as f64 / 1_000_000.0;
                let h_mm = height_nm as f64 / 1_000_000.0;
                format!("%ADD{}R,{:.4}X{:.4}*%\n", code, w_mm, h_mm)
            }
            Self::Obround {
                width_nm,
                height_nm,
            } => {
                let w_mm = width_nm as f64 / 1_000_000.0;
                let h_mm = height_nm as f64 / 1_000_000.0;
                format!("%ADD{}O,{:.4}X{:.4}*%\n", code, w_mm, h_mm)
            }
        }
    }
}

/// Primitive drawing or flashing operation in Gerber.
#[derive(Clone, Debug, PartialEq)]
enum GerberPrimitive {
    Flash { pos: Point, aperture: Aperture, rotation: f64 },
    Draw {
        start: Point,
        end: Point,
        aperture: Aperture,
    },
}

impl GerberPrimitive {
    pub fn aperture(&self) -> Aperture {
        match *self {
            Self::Flash { aperture, .. } => aperture,
            Self::Draw { aperture, .. } => aperture,
        }
    }
    fn aperture_key(&self) -> (Aperture,i64) {
        let rotation=match self { Self::Flash{rotation,aperture,..} if !matches!(aperture,Aperture::Circle{..}) => (*rotation*1_000_000.0).round() as i64, _=>0 };
        (self.aperture(),rotation)
    }

}

/// Scales a millimeter coordinate into a 10^6 integer Gerber coordinate string.
#[inline]
fn format_coord_val(v: f64) -> i64 {
    (v * 1_000_000.0).round() as i64
}

// Orthogonal flashes already have their aperture dimensions transposed.
fn flash_rotation(rotation: f64) -> f64 {
    let angle = rotation.rem_euclid(360.0);
    if (angle / 90.0 - (angle / 90.0).round()).abs() < 1e-9 { 0.0 } else { angle }
}

// Explicit aperture macros work with readers that do not implement Gerber LR.
fn rotated_aperture(ap:Aperture,angle:f64,code:u32)->String {
    let (w,h,rounded)=match ap {
        Aperture::Rect{width_nm,height_nm}=>(width_nm as f64/1e6,height_nm as f64/1e6,false),
        Aperture::Obround{width_nm,height_nm}=>(width_nm as f64/1e6,height_nm as f64/1e6,true),
        _=>return ap.format_add_directive(code),
    };
    let mut out=format!("%AMNS{code}*\n");
    if rounded {
        let diameter=w.min(h);let dx=(w-diameter)/2.0;let dy=(h-diameter)/2.0;
        if w>h {out.push_str(&format!("21,1,{:.6},{h:.6},0,0,{angle:.6}*\n",w-h));}
        if h>w {out.push_str(&format!("21,1,{w:.6},{:.6},0,0,{angle:.6}*\n",h-w));}
        let center=Point::new(dx,dy).rotate(angle);
        for sign in [-1.0,1.0] {out.push_str(&format!("1,1,{diameter:.6},{:.6},{:.6},0*\n",center.x*sign,center.y*sign));}
    } else {out.push_str(&format!("21,1,{w:.6},{h:.6},0,0,{angle:.6}*\n"));}
    out.push_str(&format!("%\n%ADD{code}NS{code}*%\n"));out
}

/// Exports a single layer of `PcbBoard` into a standard RS-274X Gerber file string.
pub fn export_gerber_layer(board: &PcbBoard, layer: Layer) -> String {
    let mut primitives: Vec<GerberPrimitive> = Vec::new();

    match layer {
        Layer::FCu => {
            // 1. Tracks on F.Cu
            for track in &board.tracks {
                if track.layer == Layer::FCu && track.width > 0.0 {
                    primitives.push(GerberPrimitive::Draw {
                        start: track.start,
                        end: track.end,
                        aperture: Aperture::circle(track.width),
                    });
                }
            }

            // 2. Vias connecting F.Cu
            for via in &board.vias {
                if via.connects_layer(Layer::FCu) && via.size > 0.0 {
                    primitives.push(GerberPrimitive::Flash {
                        pos: via.at,
                        aperture: Aperture::circle(via.size),
                        rotation: 0.0,
                    });
                }
            }

            // 3. Footprint Pads on F.Cu
            for fp in &board.footprints {
                for pad in &fp.pads {
                    let pos = fp.pad_world_position(pad);
                    match pad.pad_type {
                        PadType::Smd { layer: pad_layer } if pad_layer == Layer::FCu => {
                            let ap = Aperture::from_pad_shape(&pad.shape, fp.rotation);
                            primitives.push(GerberPrimitive::Flash { pos, aperture: ap, rotation: flash_rotation(fp.rotation) });
                        }
                        PadType::ThroughHole { outer_diameter, .. } => {
                            let ap = Aperture::from_tht_pad(&pad.shape, outer_diameter, fp.rotation);
                            primitives.push(GerberPrimitive::Flash { pos, aperture: ap, rotation: flash_rotation(fp.rotation) });
                        }
                        _ => {}
                    }
                }
            }
        }

        Layer::BCu => {
            // 1. Tracks on B.Cu
            for track in &board.tracks {
                if track.layer == Layer::BCu && track.width > 0.0 {
                    primitives.push(GerberPrimitive::Draw {
                        start: track.start,
                        end: track.end,
                        aperture: Aperture::circle(track.width),
                    });
                }
            }

            // 2. Vias connecting B.Cu
            for via in &board.vias {
                if via.connects_layer(Layer::BCu) && via.size > 0.0 {
                    primitives.push(GerberPrimitive::Flash {
                        pos: via.at,
                        aperture: Aperture::circle(via.size),
                        rotation: 0.0,
                    });
                }
            }

            // 3. Footprint Pads on B.Cu
            for fp in &board.footprints {
                for pad in &fp.pads {
                    let pos = fp.pad_world_position(pad);
                    match pad.pad_type {
                        PadType::Smd { layer: pad_layer } if pad_layer == Layer::BCu => {
                            let ap = Aperture::from_pad_shape(&pad.shape, fp.rotation);
                            primitives.push(GerberPrimitive::Flash { pos, aperture: ap, rotation: flash_rotation(fp.rotation) });
                        }
                        PadType::ThroughHole { outer_diameter, .. } => {
                            let ap = Aperture::from_tht_pad(&pad.shape, outer_diameter, fp.rotation);
                            primitives.push(GerberPrimitive::Flash { pos, aperture: ap, rotation: flash_rotation(fp.rotation) });
                        }
                        _ => {}
                    }
                }
            }
        }

        Layer::FSilkscreen => {
            let stroke_ap = Aperture::circle(0.12);
            for fp in &board.footprints {
                for (p1, p2) in crate::pcb::kicad::cleared_silkscreen(board, fp) {
                    primitives.push(GerberPrimitive::Draw {
                        start: p1,
                        end: p2,
                        aperture: stroke_ap,
                    });
                }
            }
        }

        Layer::EdgeCuts => {
            let stroke_ap = Aperture::circle(0.15);
            // Closed board boundary perimeter
            for &(p1, p2) in &board.edge_cuts().segments {
                primitives.push(GerberPrimitive::Draw {
                    start: p1,
                    end: p2,
                    aperture: stroke_ap,
                });
            }
            // Explicit graphical lines or tracks on EdgeCuts layer
            for track in &board.tracks {
                if track.layer == Layer::EdgeCuts && track.width > 0.0 {
                    primitives.push(GerberPrimitive::Draw {
                        start: track.start,
                        end: track.end,
                        aperture: Aperture::circle(track.width),
                    });
                }
            }
        }

        Layer::FMask => {
            // Solder mask openings on Top
            for fp in &board.footprints {
                for pad in &fp.pads {
                    let pos = fp.pad_world_position(pad);
                    match pad.pad_type {
                        PadType::Smd { layer: pad_layer } if pad_layer == Layer::FCu => {
                            let ap = Aperture::from_pad_shape(&pad.shape, fp.rotation);
                            primitives.push(GerberPrimitive::Flash { pos, aperture: ap, rotation: flash_rotation(fp.rotation) });
                        }
                        PadType::ThroughHole { outer_diameter, .. } => {
                            let ap = Aperture::from_tht_pad(&pad.shape, outer_diameter, fp.rotation);
                            primitives.push(GerberPrimitive::Flash { pos, aperture: ap, rotation: flash_rotation(fp.rotation) });
                        }
                        _ => {}
                    }
                }
            }
        }

        Layer::BMask => {
            // Solder mask openings on Bottom
            for fp in &board.footprints {
                for pad in &fp.pads {
                    let pos = fp.pad_world_position(pad);
                    match pad.pad_type {
                        PadType::Smd { layer: pad_layer } if pad_layer == Layer::BCu => {
                            let ap = Aperture::from_pad_shape(&pad.shape, fp.rotation);
                            primitives.push(GerberPrimitive::Flash { pos, aperture: ap, rotation: flash_rotation(fp.rotation) });
                        }
                        PadType::ThroughHole { outer_diameter, .. } => {
                            let ap = Aperture::from_tht_pad(&pad.shape, outer_diameter, fp.rotation);
                            primitives.push(GerberPrimitive::Flash { pos, aperture: ap, rotation: flash_rotation(fp.rotation) });
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    // Deduplicate apertures into a sorted dictionary starting at D10
    let mut unique_apertures: BTreeSet<(Aperture,i64)> = BTreeSet::new();
    for prim in &primitives {
        unique_apertures.insert(prim.aperture_key());
    }

    let mut aperture_map: BTreeMap<(Aperture,i64), u32> = BTreeMap::new();
    let mut current_dcode = 10u32;
    for ap in unique_apertures {
        aperture_map.insert(ap, current_dcode);
        current_dcode += 1;
    }

    // Build RS-274X Gerber content
    let mut out = String::with_capacity(8192);

    // 1. Headers
    out.push_str("G04 Netlist Shatter RS-274X Gerber Exporter*\n");
    out.push_str(&format!("G04 Layer: {}*\n", layer.kicad_name()));
    out.push_str("%FSLAX46Y46*%\n");
    out.push_str("%MOMM*%\n");
    out.push_str("%LPD*%\n");

    // 2. Aperture Table Definition
    for (ap, &dcode) in &aperture_map {
        if ap.1==0 {out.push_str(&ap.0.format_add_directive(dcode));}
        else {out.push_str(&rotated_aperture(ap.0,-(ap.1 as f64)/1e6,dcode));}
    }

    // 3. Geometry Body (Grouped by Aperture for clean CAM processing)
    for (ap, &dcode) in &aperture_map {
        out.push_str(&format!("D{}*\n", dcode));
        for prim in &primitives {
            if prim.aperture_key() == *ap {
                match *prim {
                    GerberPrimitive::Flash { pos, .. } => {
                        let x = format_coord_val(pos.x);
                        let y = format_coord_val(board.height - pos.y);
                        out.push_str(&format!("X{}Y{}D03*\n", x, y));
                    }
                    GerberPrimitive::Draw { start, end, .. } => {
                        let x1 = format_coord_val(start.x);
                        let y1 = format_coord_val(board.height - start.y);
                        let x2 = format_coord_val(end.x);
                        let y2 = format_coord_val(board.height - end.y);
                        out.push_str(&format!("X{}Y{}D02*\n", x1, y1));
                        out.push_str(&format!("X{}Y{}D01*\n", x2, y2));
                    }
                }
            }
        }
    }

    // 4. End of File
    out.push_str("M02*\n");

    out
}

/// Generates all 6 standard fabrication Gerber layers plus the Excellon drill file.
///
/// Returns a map with standard filenames:
/// - `"F_Cu.gbr"`
/// - `"B_Cu.gbr"`
/// - `"F_SilkS.gbr"`
/// - `"Edge_Cuts.gbr"`
/// - `"F_Mask.gbr"`
/// - `"B_Mask.gbr"`
/// - `"drill.drl"`
pub fn export_all_gerbers_and_drill(board: &PcbBoard) -> HashMap<String, String> {
    let mut files = HashMap::new();
    files.insert(
        Layer::FCu.gerber_filename().to_string(),
        export_gerber_layer(board, Layer::FCu),
    );
    files.insert(
        Layer::BCu.gerber_filename().to_string(),
        export_gerber_layer(board, Layer::BCu),
    );
    files.insert(
        Layer::FSilkscreen.gerber_filename().to_string(),
        export_gerber_layer(board, Layer::FSilkscreen),
    );
    files.insert(
        Layer::EdgeCuts.gerber_filename().to_string(),
        export_gerber_layer(board, Layer::EdgeCuts),
    );
    files.insert(
        Layer::FMask.gerber_filename().to_string(),
        export_gerber_layer(board, Layer::FMask),
    );
    files.insert(
        Layer::BMask.gerber_filename().to_string(),
        export_gerber_layer(board, Layer::BMask),
    );
    files.insert("drill.drl".to_string(), export_excellon_drill(board));
    files
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pcb::core::{Point, TrackSegment, Via};
    use crate::pcb::footprint::{header_1x15, resistor_0805};

    #[test]
    fn test_gerber_output() {
        let mut board = PcbBoard::new(60.0, 35.0);
        let r1 = resistor_0805("R1", "10k").at_position(Point::new(15.0, 20.0));
        board.add_footprint(r1);

        let j1 = header_1x15("J1", "CONN_01x15").at_position(Point::new(10.0, 5.0));
        board.add_footprint(j1);

        board.add_track(TrackSegment::new(
            Point::new(1.0, 2.0),
            Point::new(3.0, 4.0),
            0.25,
            Layer::FCu,
            1,
        ));
        board.add_via(Via::new(Point::new(25.0, 20.0), 0.8, 0.4, 1));

        let fcu = export_gerber_layer(&board, Layer::FCu);
        assert!(fcu.contains("%FSLAX46Y46*%"));
        assert!(fcu.contains("%MOMM*%"));
        assert!(fcu.contains("%LPD*%"));
        assert!(fcu.contains("%ADD10"));
        assert!(fcu.contains("C,"));
        assert!(fcu.contains("R,"));
        assert!(fcu.contains("D01*"));
        assert!(fcu.contains("D02*"));
        assert!(fcu.contains("D03*"));
        assert!(fcu.contains("X1000000Y33000000"));
        assert!(fcu.trim_end().ends_with("M02*"));

        let silk = export_gerber_layer(&board, Layer::FSilkscreen);
        assert!(silk.contains("D01*"));

        let edge = export_gerber_layer(&board, Layer::EdgeCuts);
        assert!(edge.contains("D01*"));

        let mask = export_gerber_layer(&board, Layer::FMask);
        assert!(mask.contains("D03*"));
    }
}
