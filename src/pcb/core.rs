//! Foundational PCB geometric primitives, coordinate transformations,
//! schematic netlist ingestion, and board layout validation.

use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

pub const EPSILON: f64 = 1e-6; // 1 nm tolerance in millimeter units

// ============================================================================
// Trigonometric & Geometric Snapping
// ============================================================================

/// Computes (sin, cos) of an angle in degrees with exact snapping for orthogonal angles
/// (0°, 90°, 180°, 270°, and multi-turn multiples) to eliminate floating-point drift.
#[inline]
pub fn sin_cos_deg(angle_deg: f64) -> (f64, f64) {
    let norm = ((angle_deg % 360.0) + 360.0) % 360.0;
    if norm < 1e-9 || (norm - 360.0).abs() < 1e-9 {
        (0.0, 1.0)
    } else if (norm - 90.0).abs() < 1e-9 {
        (1.0, 0.0)
    } else if (norm - 180.0).abs() < 1e-9 {
        (0.0, -1.0)
    } else if (norm - 270.0).abs() < 1e-9 {
        (-1.0, 0.0)
    } else {
        let rad = norm.to_radians();
        (rad.sin(), rad.cos())
    }
}

// ============================================================================
// Point: Canonical Millimeter 2D Vector
// ============================================================================

/// 2D Cartesian point in millimeter space.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    #[inline]
    pub fn distance_to(&self, other: Point) -> f64 {
        self.distance_sq(other).sqrt()
    }

    #[inline]
    pub fn distance_sq(&self, other: Point) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx * dx + dy * dy
    }

    #[inline]
    pub fn length(&self) -> f64 {
        self.length_sq().sqrt()
    }

    #[inline]
    pub fn length_sq(&self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    #[inline]
    pub fn normalize(&self) -> Point {
        let len = self.length();
        if len > 1e-12 {
            Point::new(self.x / len, self.y / len)
        } else {
            Point::zero()
        }
    }

    #[inline]
    pub fn scale(&self, factor: f64) -> Point {
        Point::new(self.x * factor, self.y * factor)
    }

    #[inline]
    pub fn dot(&self, other: Point) -> f64 {
        self.x * other.x + self.y * other.y
    }

    #[inline]
    pub fn cross(&self, other: Point) -> f64 {
        self.x * other.y - self.y * other.x
    }

    /// Rotates point around origin (0, 0) by `angle_deg` degrees counter-clockwise.
    #[inline]
    pub fn rotate(&self, angle_deg: f64) -> Point {
        let (sin_a, cos_a) = sin_cos_deg(angle_deg);
        Point {
            x: self.x * cos_a - self.y * sin_a,
            y: self.x * sin_a + self.y * cos_a,
        }
    }

    /// Rotates point around a specified center point by `angle_deg` degrees counter-clockwise.
    #[inline]
    pub fn rotate_around(&self, center: Point, angle_deg: f64) -> Point {
        let (sin_a, cos_a) = sin_cos_deg(angle_deg);
        let dx = self.x - center.x;
        let dy = self.y - center.y;
        Point {
            x: center.x + dx * cos_a - dy * sin_a,
            y: center.y + dx * sin_a + dy * cos_a,
        }
    }

    #[inline]
    pub fn lerp(&self, other: Point, t: f64) -> Point {
        Point {
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
        }
    }

    #[inline]
    pub fn midpoint(&self, other: Point) -> Point {
        self.lerp(other, 0.5)
    }

    #[inline]
    pub fn approx_eq(&self, other: Point, epsilon: f64) -> bool {
        (self.x - other.x).abs() <= epsilon && (self.y - other.y).abs() <= epsilon
    }
}

impl Add for Point {
    type Output = Point;
    #[inline]
    fn add(self, rhs: Point) -> Point {
        Point::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl AddAssign for Point {
    #[inline]
    fn add_assign(&mut self, rhs: Point) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Sub for Point {
    type Output = Point;
    #[inline]
    fn sub(self, rhs: Point) -> Point {
        Point::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl SubAssign for Point {
    #[inline]
    fn sub_assign(&mut self, rhs: Point) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl Mul<f64> for Point {
    type Output = Point;
    #[inline]
    fn mul(self, rhs: f64) -> Point {
        Point::new(self.x * rhs, self.y * rhs)
    }
}

impl MulAssign<f64> for Point {
    #[inline]
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl Div<f64> for Point {
    type Output = Point;
    #[inline]
    fn div(self, rhs: f64) -> Point {
        if rhs.abs() > 1e-12 {
            Point::new(self.x / rhs, self.y / rhs)
        } else {
            Point::zero()
        }
    }
}

impl DivAssign<f64> for Point {
    #[inline]
    fn div_assign(&mut self, rhs: f64) {
        if rhs.abs() > 1e-12 {
            self.x /= rhs;
            self.y /= rhs;
        } else {
            self.x = 0.0;
            self.y = 0.0;
        }
    }
}

impl Neg for Point {
    type Output = Point;
    #[inline]
    fn neg(self) -> Point {
        Point::new(-self.x, -self.y)
    }
}

// ============================================================================
// Rect: 2D Axis-Aligned Bounding Box (AABB)
// ============================================================================

/// 2D Axis-Aligned Bounding Box (AABB) in millimeter space.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Rect {
    pub min: Point,
    pub max: Point,
}

impl Rect {
    pub fn new(p1: Point, p2: Point) -> Self {
        Self {
            min: Point::new(p1.x.min(p2.x), p1.y.min(p2.y)),
            max: Point::new(p1.x.max(p2.x), p1.y.max(p2.y)),
        }
    }

    pub fn from_coords(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Self {
        Self {
            min: Point::new(min_x.min(max_x), min_y.min(max_y)),
            max: Point::new(min_x.max(max_x), min_y.max(max_y)),
        }
    }

    pub fn from_center_size(center: Point, width: f64, height: f64) -> Self {
        let half_w = width / 2.0;
        let half_h = height / 2.0;
        Self {
            min: Point::new(center.x - half_w, center.y - half_h),
            max: Point::new(center.x + half_w, center.y + half_h),
        }
    }

    pub const fn from_point(p: Point) -> Self {
        Self { min: p, max: p }
    }

    #[inline]
    pub fn width(&self) -> f64 {
        self.max.x - self.min.x
    }

    #[inline]
    pub fn height(&self) -> f64 {
        self.max.y - self.min.y
    }

    #[inline]
    pub fn center(&self) -> Point {
        Point::new((self.min.x + self.max.x) / 2.0, (self.min.y + self.max.y) / 2.0)
    }

    #[inline]
    pub fn area(&self) -> f64 {
        self.width() * self.height()
    }

    #[inline]
    pub fn contains_point(&self, p: Point) -> bool {
        p.x >= self.min.x - 1e-9
            && p.x <= self.max.x + 1e-9
            && p.y >= self.min.y - 1e-9
            && p.y <= self.max.y + 1e-9
    }

    #[inline]
    pub fn intersects(&self, other: &Rect) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
    }

    pub fn union(&self, other: &Rect) -> Rect {
        Rect {
            min: Point::new(self.min.x.min(other.min.x), self.min.y.min(other.min.y)),
            max: Point::new(self.max.x.max(other.max.x), self.max.y.max(other.max.y)),
        }
    }

    pub fn inflate(&self, amount: f64) -> Rect {
        Rect {
            min: Point::new(self.min.x - amount, self.min.y - amount),
            max: Point::new(self.max.x + amount, self.max.y + amount),
        }
    }
}

// ============================================================================
// Layer & NetId
// ============================================================================

/// PCB Layer enumeration matching KiCad and Gerber CAM standards.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Layer {
    FCu,
    BCu,
    FSilkscreen,
    EdgeCuts,
    FMask,
    BMask,
}

impl Layer {
    #[inline]
    pub fn is_copper(&self) -> bool {
        matches!(self, Layer::FCu | Layer::BCu)
    }

    #[inline]
    pub fn kicad_name(&self) -> &'static str {
        match self {
            Layer::FCu => "F.Cu",
            Layer::BCu => "B.Cu",
            Layer::FSilkscreen => "F.SilkS",
            Layer::EdgeCuts => "Edge.Cuts",
            Layer::FMask => "F.Mask",
            Layer::BMask => "B.Mask",
        }
    }

    pub fn from_kicad_name(name: &str) -> Option<Layer> {
        match name {
            "F.Cu" => Some(Layer::FCu),
            "B.Cu" => Some(Layer::BCu),
            "F.SilkS" => Some(Layer::FSilkscreen),
            "Edge.Cuts" => Some(Layer::EdgeCuts),
            "F.Mask" => Some(Layer::FMask),
            "B.Mask" => Some(Layer::BMask),
            _ => None,
        }
    }

    #[inline]
    pub fn gerber_filename(&self) -> &'static str {
        match self {
            Layer::FCu => "F_Cu.gbr",
            Layer::BCu => "B_Cu.gbr",
            Layer::FSilkscreen => "F_SilkS.gbr",
            Layer::EdgeCuts => "Edge_Cuts.gbr",
            Layer::FMask => "F_Mask.gbr",
            Layer::BMask => "B_Mask.gbr",
        }
    }
}

/// Canonical Net Identifier type.
pub type NetId = u32;

/// Net ID 0 represents unassigned/unconnected nets or non-electrical geometry.
pub const UNCONNECTED_NET: NetId = 0;

// ============================================================================
// Pad Shape & Pad Type
// ============================================================================

/// Geometric shape of copper pads.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum PadShape {
    Rect { width: f64, height: f64 },
    Circle { diameter: f64 },
    Obround { width: f64, height: f64 },
}

impl PadShape {
    /// Computes the exact axis-aligned bounding box (AABB) of the shape centered
    /// at `center` with rotation `rotation_deg`.
    pub fn bounding_box(&self, center: Point, rotation_deg: f64) -> Rect {
        match *self {
            PadShape::Circle { diameter } => Rect::from_center_size(center, diameter, diameter),
            PadShape::Rect { width, height } => {
                let (sin_a, cos_a) = sin_cos_deg(rotation_deg);
                let half_w = width / 2.0;
                let half_h = height / 2.0;
                let dx = half_w * cos_a.abs() + half_h * sin_a.abs();
                let dy = half_w * sin_a.abs() + half_h * cos_a.abs();
                Rect {
                    min: Point::new(center.x - dx, center.y - dy),
                    max: Point::new(center.x + dx, center.y + dy),
                }
            }
            PadShape::Obround { width, height } => {
                let (sin_a, cos_a) = sin_cos_deg(rotation_deg);
                if width >= height {
                    let spine_half = (width - height) / 2.0;
                    let radius = height / 2.0;
                    let dx = spine_half * cos_a.abs() + radius;
                    let dy = spine_half * sin_a.abs() + radius;
                    Rect {
                        min: Point::new(center.x - dx, center.y - dy),
                        max: Point::new(center.x + dx, center.y + dy),
                    }
                } else {
                    let spine_half = (height - width) / 2.0;
                    let radius = width / 2.0;
                    let dx = spine_half * sin_a.abs() + radius;
                    let dy = spine_half * cos_a.abs() + radius;
                    Rect {
                        min: Point::new(center.x - dx, center.y - dy),
                        max: Point::new(center.x + dx, center.y + dy),
                    }
                }
            }
        }
    }

    #[inline]
    pub fn kicad_shape_name(&self) -> &'static str {
        match self {
            PadShape::Rect { .. } => "rect",
            PadShape::Circle { .. } => "circle",
            PadShape::Obround { .. } => "oval",
        }
    }
}

/// Pad mounting technology and electrical layer assignment.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum PadType {
    Smd { layer: Layer },
    ThroughHole { drill: f64, outer_diameter: f64 },
}

impl PadType {
    #[inline]
    pub fn is_smd(&self) -> bool {
        matches!(self, PadType::Smd { .. })
    }

    #[inline]
    pub fn is_through_hole(&self) -> bool {
        matches!(self, PadType::ThroughHole { .. })
    }

    #[inline]
    pub fn drill_diameter(&self) -> Option<f64> {
        match self {
            PadType::ThroughHole { drill, .. } => Some(*drill),
            PadType::Smd { .. } => None,
        }
    }

    #[inline]
    pub fn annular_ring(&self) -> Option<f64> {
        match self {
            PadType::ThroughHole { drill, outer_diameter } => {
                Some((outer_diameter - drill) / 2.0)
            }
            PadType::Smd { .. } => None,
        }
    }
}

// ============================================================================
// Pad
// ============================================================================

/// Physical pad definition belonging to a component footprint.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pad {
    pub pin_number: String,
    pub pin_name: String,
    pub relative_pos: Point,
    pub shape: PadShape,
    pub pad_type: PadType,
    pub net: Option<NetId>,
}

impl Pad {
    pub fn new_smd(
        pin_number: impl Into<String>,
        pin_name: impl Into<String>,
        relative_pos: Point,
        shape: PadShape,
        layer: Layer,
    ) -> Self {
        Self {
            pin_number: pin_number.into(),
            pin_name: pin_name.into(),
            relative_pos,
            shape,
            pad_type: PadType::Smd { layer },
            net: None,
        }
    }

    pub fn new_tht(
        pin_number: impl Into<String>,
        pin_name: impl Into<String>,
        relative_pos: Point,
        shape: PadShape,
        drill: f64,
        outer_diameter: f64,
    ) -> Self {
        Self {
            pin_number: pin_number.into(),
            pin_name: pin_name.into(),
            relative_pos,
            shape,
            pad_type: PadType::ThroughHole { drill, outer_diameter },
            net: None,
        }
    }

    pub fn with_net(mut self, net: NetId) -> Self {
        self.net = Some(net);
        self
    }

    #[inline]
    pub fn global_position(&self, footprint_pos: Point, footprint_rot: f64) -> Point {
        footprint_pos + self.relative_pos.rotate(footprint_rot)
    }

    pub fn bounding_box(&self, footprint_pos: Point, footprint_rot: f64) -> Rect {
        let center = self.global_position(footprint_pos, footprint_rot);
        self.shape.bounding_box(center, footprint_rot)
    }

    #[inline]
    pub fn annular_ring(&self) -> Option<f64> {
        self.pad_type.annular_ring()
    }
}

// ============================================================================
// TrackSegment & Via
// ============================================================================

/// Copper track segment.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrackSegment {
    pub start: Point,
    pub end: Point,
    pub width: f64,
    pub layer: Layer,
    pub net: NetId,
}

pub type Track = TrackSegment;

impl TrackSegment {
    pub fn new(start: Point, end: Point, width: f64, layer: Layer, net: NetId) -> Self {
        Self { start, end, width, layer, net }
    }

    #[inline]
    pub fn length(&self) -> f64 {
        self.start.distance_to(self.end)
    }

    #[inline]
    pub fn midpoint(&self) -> Point {
        self.start.midpoint(self.end)
    }

    pub fn bounding_box(&self) -> Rect {
        let half_w = self.width / 2.0;
        let min_x = self.start.x.min(self.end.x) - half_w;
        let max_x = self.start.x.max(self.end.x) + half_w;
        let min_y = self.start.y.min(self.end.y) - half_w;
        let max_y = self.start.y.max(self.end.y) + half_w;
        Rect::new(Point::new(min_x, min_y), Point::new(max_x, max_y))
    }

    pub fn distance_to_point(&self, p: Point) -> f64 {
        let v = self.end - self.start;
        let len_sq = v.length_sq();
        if len_sq < 1e-12 {
            return self.start.distance_to(p);
        }
        let t = ((p - self.start).dot(v) / len_sq).clamp(0.0, 1.0);
        let proj = self.start + v * t;
        proj.distance_to(p)
    }
}

/// Plated through-hole via connecting copper layers.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Via {
    pub at: Point,
    pub size: f64,   // outer pad diameter (mm)
    pub drill: f64,  // drill hole diameter (mm)
    pub net: NetId,
    pub layers: (Layer, Layer),
}

impl Via {
    pub fn new(at: Point, size: f64, drill: f64, net: NetId) -> Self {
        Self {
            at,
            size,
            drill,
            net,
            layers: (Layer::FCu, Layer::BCu),
        }
    }

    pub fn with_layers(at: Point, size: f64, drill: f64, net: NetId, layers: (Layer, Layer)) -> Self {
        Self { at, size, drill, net, layers }
    }

    #[inline]
    pub fn annular_ring(&self) -> f64 {
        (self.size - self.drill) / 2.0
    }

    #[inline]
    pub fn bounding_box(&self) -> Rect {
        Rect::from_center_size(self.at, self.size, self.size)
    }

    #[inline]
    pub fn connects_layer(&self, layer: Layer) -> bool {
        self.layers.0 == layer || self.layers.1 == layer
    }
}

// ============================================================================
// Footprint: Physical Component Instance
// ============================================================================

/// Placed component footprint instance.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Footprint {
    pub reference: String,
    pub value: String,
    pub package: String,
    pub position: Point,
    pub rotation: f64, // degrees
    pub pads: Vec<Pad>,
    pub silkscreen_lines: Vec<(Point, Point)>,
}

impl Footprint {
    pub fn new(
        reference: impl Into<String>,
        value: impl Into<String>,
        package: impl Into<String>,
        position: Point,
        rotation: f64,
        pads: Vec<Pad>,
        silkscreen_lines: Vec<(Point, Point)>,
    ) -> Self {
        Self {
            reference: reference.into(),
            value: value.into(),
            package: package.into(),
            position,
            rotation,
            pads,
            silkscreen_lines,
        }
    }

    pub fn pad(&self, pin_number: &str) -> Option<&Pad> {
        self.pads
            .iter()
            .find(|p| p.pin_number == pin_number || p.pin_name == pin_number)
    }

    pub fn pad_mut(&mut self, pin_number: &str) -> Option<&mut Pad> {
        self.pads
            .iter_mut()
            .find(|p| p.pin_number == pin_number || p.pin_name == pin_number)
    }

    pub fn at_position(mut self, pos: Point) -> Self {
        self.position = pos;
        self
    }

    pub fn with_rotation(mut self, rot_deg: f64) -> Self {
        self.rotation = rot_deg;
        self
    }

    pub fn with_net_assigned(mut self, pin_number: &str, net: NetId) -> Self {
        if let Some(pad) = self.pad_mut(pin_number) {
            pad.net = Some(net);
        }
        self
    }

    pub fn pad_world_pos(&self, pin_number: &str) -> Option<Point> {
        self.pad(pin_number)
            .map(|p| p.global_position(self.position, self.rotation))
    }

    pub fn pad_world_position(&self, pad: &Pad) -> Point {
        pad.global_position(self.position, self.rotation)
    }

    pub fn pad_world_bbox(&self, pad: &Pad) -> Rect {
        pad.bounding_box(self.position, self.rotation)
    }

    pub fn all_pad_world_positions(&self) -> Vec<(&Pad, Point)> {
        self.pads
            .iter()
            .map(|p| (p, p.global_position(self.position, self.rotation)))
            .collect()
    }

    pub fn silkscreen_world_lines(&self) -> Vec<(Point, Point)> {
        self.silkscreen_lines
            .iter()
            .map(|&(p1, p2)| {
                (
                    self.position + p1.rotate(self.rotation),
                    self.position + p2.rotate(self.rotation),
                )
            })
            .collect()
    }

    pub fn world_bounding_box(&self) -> Rect {
        let mut aabb: Option<Rect> = None;
        for pad in &self.pads {
            let pad_box = pad.bounding_box(self.position, self.rotation);
            aabb = Some(match aabb {
                Some(current) => current.union(&pad_box),
                None => pad_box,
            });
        }
        for (p1, p2) in self.silkscreen_world_lines() {
            let line_box = Rect::new(p1, p2);
            aabb = Some(match aabb {
                Some(current) => current.union(&line_box),
                None => line_box,
            });
        }
        aabb.unwrap_or_else(|| Rect::from_point(self.position))
    }

    #[inline]
    pub fn bounding_box(&self) -> Rect {
        self.world_bounding_box()
    }

    pub fn assign_net(&mut self, pin_number: &str, net: NetId) -> bool {
        if let Some(pad) = self.pad_mut(pin_number) {
            pad.net = Some(net);
            true
        } else {
            false
        }
    }

    pub fn validate(&self) -> Vec<ValidationError> {
        let mut errors = Vec::new();

        if self.pads.is_empty() {
            errors.push(ValidationError::EmptyFootprint {
                component_ref: self.reference.clone(),
            });
            return errors;
        }

        if !self.position.x.is_finite() || !self.position.y.is_finite() || !self.rotation.is_finite() {
            errors.push(ValidationError::NonFiniteCoordinate {
                component_ref: self.reference.clone(),
                pin_number: None,
                details: format!(
                    "Position {:?} or rotation {} is non-finite",
                    self.position, self.rotation
                ),
            });
        }

        let mut seen_pins = HashSet::new();
        for pad in &self.pads {
            if !seen_pins.insert(&pad.pin_number) {
                errors.push(ValidationError::DuplicatePinNumber {
                    component_ref: self.reference.clone(),
                    pin_number: pad.pin_number.clone(),
                });
            }

            if !pad.relative_pos.x.is_finite() || !pad.relative_pos.y.is_finite() {
                errors.push(ValidationError::NonFiniteCoordinate {
                    component_ref: self.reference.clone(),
                    pin_number: Some(pad.pin_number.clone()),
                    details: format!("Relative pos {:?} is non-finite", pad.relative_pos),
                });
            }

            match pad.shape {
                PadShape::Rect { width, height } | PadShape::Obround { width, height } => {
                    if !width.is_finite() || width <= 0.0 || !height.is_finite() || height <= 0.0 {
                        errors.push(ValidationError::InvalidPadDimensions {
                            component_ref: self.reference.clone(),
                            pin_number: pad.pin_number.clone(),
                            details: format!(
                                "Width ({}) and height ({}) must be positive finite values",
                                width, height
                            ),
                        });
                    }
                }
                PadShape::Circle { diameter } => {
                    if !diameter.is_finite() || diameter <= 0.0 {
                        errors.push(ValidationError::InvalidPadDimensions {
                            component_ref: self.reference.clone(),
                            pin_number: pad.pin_number.clone(),
                            details: format!(
                                "Diameter ({}) must be a positive finite value",
                                diameter
                            ),
                        });
                    }
                }
            }

            match pad.pad_type {
                PadType::Smd { layer } => {
                    if layer != Layer::FCu && layer != Layer::BCu {
                        errors.push(ValidationError::InvalidPadLayer {
                            component_ref: self.reference.clone(),
                            pin_number: pad.pin_number.clone(),
                            layer,
                        });
                    }
                }
                PadType::ThroughHole { drill, outer_diameter } => {
                    if !drill.is_finite() || drill <= 0.0 {
                        errors.push(ValidationError::InvalidThroughHoleDrill {
                            component_ref: self.reference.clone(),
                            pin_number: pad.pin_number.clone(),
                            drill,
                            outer_diameter,
                            details: format!("Drill diameter ({}) must be positive finite", drill),
                        });
                    } else if !outer_diameter.is_finite() || outer_diameter <= drill {
                        errors.push(ValidationError::InvalidThroughHoleDrill {
                            component_ref: self.reference.clone(),
                            pin_number: pad.pin_number.clone(),
                            drill,
                            outer_diameter,
                            details: format!(
                                "Outer diameter ({}) must be strictly greater than drill ({})",
                                outer_diameter, drill
                            ),
                        });
                    }
                }
            }
        }

        errors
    }
}

// ============================================================================
// EdgeCuts: Closed Polygon Outline
// ============================================================================

/// Closed polygon outline representing the physical board edge cuts.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EdgeCuts {
    pub segments: Vec<(Point, Point)>,
}

impl EdgeCuts {
    pub fn new(segments: Vec<(Point, Point)>) -> Self {
        Self { segments }
    }

    pub fn from_rect(width: f64, height: f64) -> Self {
        Self {
            segments: vec![
                (Point::new(0.0, 0.0), Point::new(width, 0.0)),
                (Point::new(width, 0.0), Point::new(width, height)),
                (Point::new(width, height), Point::new(0.0, height)),
                (Point::new(0.0, height), Point::new(0.0, 0.0)),
            ],
        }
    }

    pub fn from_vertices(vertices: &[Point]) -> Self {
        if vertices.len() < 3 {
            return Self { segments: Vec::new() };
        }
        let mut segments = Vec::with_capacity(vertices.len());
        for i in 0..vertices.len() {
            let next = (i + 1) % vertices.len();
            segments.push((vertices[i], vertices[next]));
        }
        Self { segments }
    }

    pub fn is_closed(&self, epsilon: f64) -> bool {
        if self.segments.len() < 3 {
            return false;
        }
        for i in 0..self.segments.len() {
            let current_end = self.segments[i].1;
            let next_start = self.segments[(i + 1) % self.segments.len()].0;
            if current_end.distance_to(next_start) > epsilon {
                return false;
            }
        }
        true
    }

    pub fn bounding_box(&self) -> Rect {
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;

        for &(p1, p2) in &self.segments {
            min_x = min_x.min(p1.x).min(p2.x);
            max_x = max_x.max(p1.x).max(p2.x);
            min_y = min_y.min(p1.y).min(p2.y);
            max_y = max_y.max(p1.y).max(p2.y);
        }

        if min_x.is_infinite() {
            Rect::new(Point::zero(), Point::zero())
        } else {
            Rect::new(Point::new(min_x, min_y), Point::new(max_x, max_y))
        }
    }

    pub fn area(&self) -> f64 {
        if self.segments.len() < 3 {
            return 0.0;
        }
        let mut sum = 0.0;
        for &(p1, p2) in &self.segments {
            sum += p1.x * p2.y - p2.x * p1.y;
        }
        (sum / 2.0).abs()
    }

    #[inline]
    pub fn polygon_area(&self) -> f64 {
        self.area()
    }

    pub fn contains_point(&self, p: Point) -> bool {
        let mut inside = false;
        for &(p1, p2) in &self.segments {
            let (x1, y1) = (p1.x, p1.y);
            let (x2, y2) = (p2.x, p2.y);
            if ((y1 > p.y) != (y2 > p.y))
                && (p.x < (x2 - x1) * (p.y - y1) / (y2 - y1 + 1e-15) + x1)
            {
                inside = !inside;
            }
        }
        inside
    }

    pub fn distance_to_point(&self, p: Point) -> f64 {
        let mut min_dist = f64::INFINITY;
        for &(p1, p2) in &self.segments {
            let seg = TrackSegment::new(p1, p2, 0.0, Layer::EdgeCuts, 0);
            min_dist = min_dist.min(seg.distance_to_point(p));
        }
        min_dist
    }

    pub fn to_track_segments(&self) -> Vec<TrackSegment> {
        self.segments
            .iter()
            .map(|&(p1, p2)| TrackSegment::new(p1, p2, 0.15, Layer::EdgeCuts, 0))
            .collect()
    }
}

// ============================================================================
// Schematic Netlist Models & Ingestion
// ============================================================================

/// Target pin of a physical component footprint.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NetPinRef {
    pub component_ref: String,
    pub pin_id: String,
}

impl NetPinRef {
    pub fn new(component_ref: impl Into<String>, pin_id: impl Into<String>) -> Self {
        Self {
            component_ref: component_ref.into(),
            pin_id: pin_id.into(),
        }
    }
}

/// Logical net connecting two or more component pins.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SchematicNet {
    pub name: String,
    pub pins: Vec<NetPinRef>,
}

impl SchematicNet {
    pub fn new(name: impl Into<String>, pins: Vec<NetPinRef>) -> Self {
        Self {
            name: name.into(),
            pins,
        }
    }
}

/// Complete schematic netlist defining circuit connectivity.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SchematicNetlist {
    pub nets: Vec<SchematicNet>,
}

impl SchematicNetlist {
    pub fn new() -> Self {
        Self { nets: Vec::new() }
    }

    pub fn add_net(&mut self, name: &str, pins: &[(&str, &str)]) {
        let pin_refs = pins.iter().map(|&(c, p)| NetPinRef::new(c, p)).collect();
        self.nets.push(SchematicNet::new(name, pin_refs));
    }
}

/// Ingestion error returned when netlist pin mapping fails.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum IngestionError {
    ComponentNotFound {
        component_ref: String,
        net_name: String,
    },
    PinNotFound {
        component_ref: String,
        pin_id: String,
        net_name: String,
    },
    DuplicatePinAssignment {
        component_ref: String,
        pin_id: String,
        existing_net_name: String,
        new_net_name: String,
    },
}

impl std::fmt::Display for IngestionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ComponentNotFound {
                component_ref,
                net_name,
            } => {
                write!(
                    f,
                    "Component '{}' in net '{}' not found on board",
                    component_ref, net_name
                )
            }
            Self::PinNotFound {
                component_ref,
                pin_id,
                net_name,
            } => {
                write!(
                    f,
                    "Pin '{}' of component '{}' in net '{}' not found",
                    pin_id, component_ref, net_name
                )
            }
            Self::DuplicatePinAssignment {
                component_ref,
                pin_id,
                existing_net_name,
                new_net_name,
            } => {
                write!(
                    f,
                    "Pin '{}.{}' already assigned to net '{}', cannot assign to '{}'",
                    component_ref, pin_id, existing_net_name, new_net_name
                )
            }
        }
    }
}

impl std::error::Error for IngestionError {}

// ============================================================================
// Board Validation Models
// ============================================================================

/// Taxonomy of board placement and geometry validation errors.
#[derive(Clone, Debug, PartialEq)]
pub enum ValidationError {
    InvalidPadDimensions {
        component_ref: String,
        pin_number: String,
        details: String,
    },
    InvalidThroughHoleDrill {
        component_ref: String,
        pin_number: String,
        drill: f64,
        outer_diameter: f64,
        details: String,
    },
    InvalidPadLayer {
        component_ref: String,
        pin_number: String,
        layer: Layer,
    },
    PadOutOfBounds {
        component_ref: String,
        pin_number: String,
        pad_center: Point,
        pad_bbox: Rect,
        board_bounds: Rect,
    },
    DuplicatePinNumber {
        component_ref: String,
        pin_number: String,
    },
    EmptyFootprint {
        component_ref: String,
    },
    InvalidBoardDimensions {
        width: f64,
        height: f64,
    },
    NonFiniteCoordinate {
        component_ref: String,
        pin_number: Option<String>,
        details: String,
    },
}

/// Report summarizing physical verification of board geometry.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BoardValidationReport {
    pub errors: Vec<ValidationError>,
    pub total_footprints: usize,
    pub total_pads: usize,
    pub connected_pads: usize,
    pub unconnected_pads: usize,
}

impl BoardValidationReport {
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.errors.is_empty()
    }
}

// ============================================================================
// PcbBoard: Primary PCB Assembly
// ============================================================================

/// Container representing the physical PCB board.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PcbBoard {
    pub width: f64,
    pub height: f64,
    #[serde(default = "default_board_thickness")]
    pub thickness: f64,
    pub footprints: Vec<Footprint>,
    pub tracks: Vec<TrackSegment>,
    pub vias: Vec<Via>,
    pub net_names: HashMap<NetId, String>,
}

fn default_board_thickness() -> f64 { 1.6 }

impl PcbBoard {
    pub fn new(width: f64, height: f64) -> Self {
        let mut net_names = HashMap::new();
        net_names.insert(UNCONNECTED_NET, String::new());
        Self {
            width,
            height,
            thickness: default_board_thickness(),
            footprints: Vec::new(),
            tracks: Vec::new(),
            vias: Vec::new(),
            net_names,
        }
    }

    /// Connect every physical pad with this exact number. A missing terminal or
    /// accidental reassignment is a design error, never an ignored instruction.
    pub fn connect(&mut self, reference: &str, pin: &str, net: NetId) {
        assert!(net != UNCONNECTED_NET && self.net_names.contains_key(&net), "Unknown net {net}");
        let fp = self.find_footprint_mut(reference).unwrap_or_else(|| panic!("Missing component {reference}"));
        let mut found = false;
        for pad in fp.pads.iter_mut().filter(|p| p.pin_number == pin) {
            assert!(pad.net.is_none() || pad.net == Some(net), "Conflicting assignment for {reference}.{pin}");
            pad.net = Some(net);
            found = true;
        }
        assert!(found, "Missing pin {reference}.{pin}");
    }

    pub fn register_net(&mut self, id: NetId, name: impl Into<String>) {
        self.net_names.insert(id, name.into());
    }

    pub fn get_or_create_net_id(&mut self, name: &str) -> NetId {
        for (&id, net_name) in &self.net_names {
            if net_name == name {
                return id;
            }
        }
        let next_id = (self.net_names.keys().copied().max().unwrap_or(0) + 1).max(1);
        self.net_names.insert(next_id, name.to_string());
        next_id
    }

    #[inline]
    pub fn get_or_create_net(&mut self, name: &str) -> NetId {
        self.get_or_create_net_id(name)
    }

    pub fn get_net_name(&self, net_id: NetId) -> Option<&str> {
        self.net_names.get(&net_id).map(|s| s.as_str())
    }

    pub fn add_footprint(&mut self, footprint: Footprint) {
        self.footprints.push(footprint);
    }

    pub fn add_track(&mut self, track: TrackSegment) {
        self.tracks.push(track);
    }

    pub fn add_via(&mut self, via: Via) {
        self.vias.push(via);
    }

    pub fn edge_cuts(&self) -> EdgeCuts {
        EdgeCuts::from_rect(self.width, self.height)
    }

    pub fn find_footprint(&self, reference: &str) -> Option<&Footprint> {
        self.footprints.iter().find(|fp| fp.reference == reference)
    }

    pub fn find_footprint_mut(&mut self, reference: &str) -> Option<&mut Footprint> {
        self.footprints.iter_mut().find(|fp| fp.reference == reference)
    }

    pub fn all_pads_global(&self) -> Vec<(usize, usize, Point, &Pad)> {
        let mut pads = Vec::new();
        for (fp_idx, fp) in self.footprints.iter().enumerate() {
            for (pad_idx, pad) in fp.pads.iter().enumerate() {
                let g_pos = pad.global_position(fp.position, fp.rotation);
                pads.push((fp_idx, pad_idx, g_pos, pad));
            }
        }
        pads
    }

    pub fn pads_for_net(&self, net: NetId) -> Vec<Point> {
        let mut positions = Vec::new();
        for fp in &self.footprints {
            for pad in &fp.pads {
                if pad.net == Some(net) {
                    positions.push(pad.global_position(fp.position, fp.rotation));
                }
            }
        }
        positions
    }

    #[inline]
    pub fn bounding_box(&self) -> Rect {
        Rect::new(Point::zero(), Point::new(self.width, self.height))
    }

    #[inline]
    pub fn is_point_on_board(&self, p: Point) -> bool {
        p.x >= 0.0 && p.x <= self.width && p.y >= 0.0 && p.y <= self.height
    }

    pub fn assign_pad_net(
        &mut self,
        component_ref: &str,
        pin_id: &str,
        net_name: &str,
    ) -> Result<(), IngestionError> {
        let net_id = self.get_or_create_net_id(net_name);

        let fp = self
            .footprints
            .iter_mut()
            .find(|f| f.reference == component_ref)
            .ok_or_else(|| IngestionError::ComponentNotFound {
                component_ref: component_ref.to_string(),
                net_name: net_name.to_string(),
            })?;

        let pad = fp
            .pads
            .iter_mut()
            .find(|p| p.pin_number == pin_id || p.pin_name == pin_id)
            .ok_or_else(|| IngestionError::PinNotFound {
                component_ref: component_ref.to_string(),
                pin_id: pin_id.to_string(),
                net_name: net_name.to_string(),
            })?;

        if let Some(existing_id) = pad.net {
            if existing_id != net_id {
                let existing_net_name = self.net_names.get(&existing_id).cloned().unwrap_or_default();
                return Err(IngestionError::DuplicatePinAssignment {
                    component_ref: component_ref.to_string(),
                    pin_id: pin_id.to_string(),
                    existing_net_name,
                    new_net_name: net_name.to_string(),
                });
            }
        }

        pad.net = Some(net_id);
        Ok(())
    }

    pub fn apply_netlist(&mut self, netlist: &SchematicNetlist) -> Result<(), IngestionError> {
        for net in &netlist.nets {
            for pin_ref in &net.pins {
                self.assign_pad_net(&pin_ref.component_ref, &pin_ref.pin_id, &net.name)?;
            }
        }
        Ok(())
    }

    pub fn validate(&self) -> BoardValidationReport {
        let mut report = BoardValidationReport::default();

        if !self.width.is_finite()
            || self.width <= 0.0
            || !self.height.is_finite()
            || self.height <= 0.0
        {
            report.errors.push(ValidationError::InvalidBoardDimensions {
                width: self.width,
                height: self.height,
            });
            return report;
        }

        let board_bounds = Rect::new(Point::zero(), Point::new(self.width, self.height));
        report.total_footprints = self.footprints.len();

        for fp in &self.footprints {
            report.errors.extend(fp.validate());

            for pad in &fp.pads {
                report.total_pads += 1;
                if pad.net.is_some() {
                    report.connected_pads += 1;
                } else {
                    report.unconnected_pads += 1;
                }

                let pad_center = fp.pad_world_position(pad);
                let pad_bbox = fp.pad_world_bbox(pad);

                if pad_bbox.min.x < -1e-6
                    || pad_bbox.min.y < -1e-6
                    || pad_bbox.max.x > self.width + 1e-6
                    || pad_bbox.max.y > self.height + 1e-6
                {
                    report.errors.push(ValidationError::PadOutOfBounds {
                        component_ref: fp.reference.clone(),
                        pin_number: pad.pin_number.clone(),
                        pad_center,
                        pad_bbox,
                        board_bounds,
                    });
                }
            }
        }

        report
    }
}

// ============================================================================
// Ratsnest & Euclidean Minimum Spanning Tree (EMST)
// ============================================================================

/// Connection line in the dynamic ratsnest Euclidean Minimum Spanning Tree.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RatsnestLine {
    pub start: Point,
    pub end: Point,
    pub net: NetId,
}

impl RatsnestLine {
    pub fn new(start: Point, end: Point, net: NetId) -> Self {
        Self { start, end, net }
    }
}

/// Disjoint Set Union (Union-Find) helper.
struct DisjointSet {
    parent: Vec<usize>,
    rank: Vec<usize>,
}

impl DisjointSet {
    fn new(size: usize) -> Self {
        Self {
            parent: (0..size).collect(),
            rank: vec![0; size],
        }
    }

    fn find(&mut self, x: usize) -> usize {
        if self.parent[x] != x {
            self.parent[x] = self.find(self.parent[x]);
        }
        self.parent[x]
    }

    fn union(&mut self, x: usize, y: usize) -> bool {
        let rx = self.find(x);
        let ry = self.find(y);
        if rx == ry {
            return false;
        }
        if self.rank[rx] < self.rank[ry] {
            self.parent[rx] = ry;
        } else if self.rank[rx] > self.rank[ry] {
            self.parent[ry] = rx;
        } else {
            self.parent[ry] = rx;
            self.rank[rx] += 1;
        }
        true
    }
}

/// Computes the dynamic ratsnest Euclidean Minimum Spanning Tree (EMST) lines across
/// all multi-pin nets on the board, accounting for copper-connected island clustering.
pub fn compute_ratsnest(board: &PcbBoard) -> Vec<RatsnestLine> {
    let mut max_net_id = board.net_names.keys().copied().max().unwrap_or(0);
    for fp in &board.footprints {
        for pad in &fp.pads {
            if let Some(nid) = pad.net {
                if nid != UNCONNECTED_NET && nid > max_net_id {
                    max_net_id = nid;
                }
            }
        }
    }

    if max_net_id == 0 {
        return Vec::new();
    }

    let mut net_pads: Vec<Vec<Point>> = vec![Vec::new(); (max_net_id as usize) + 1];

    for fp in &board.footprints {
        for pad in &fp.pads {
            if let Some(net_id) = pad.net {
                if net_id != UNCONNECTED_NET {
                    let g_pos = pad.global_position(fp.position, fp.rotation);
                    net_pads[net_id as usize].push(g_pos);
                }
            }
        }
    }

    let mut ratsnest_lines = Vec::new();

    let has_copper = !board.tracks.is_empty() || !board.vias.is_empty();
    let mut tracks_by_net: HashMap<NetId, Vec<&TrackSegment>> = HashMap::new();
    let mut vias_by_net: HashMap<NetId, Vec<&Via>> = HashMap::new();
    if has_copper {
        for t in &board.tracks {
            tracks_by_net.entry(t.net).or_default().push(t);
        }
        for v in &board.vias {
            vias_by_net.entry(v.net).or_default().push(v);
        }
    }

    for (net_idx, pads) in net_pads.iter().enumerate() {
        let net_id = net_idx as NetId;
        let n_pads = pads.len();
        if n_pads <= 1 {
            continue;
        }

        if !has_copper {
            // Fast path for unrouted boards: each pad is an isolated island.
            // Directly compute Euclidean Minimum Spanning Tree (EMST) via Kruskal's algorithm.
            let mut edges: Vec<(f64, usize, usize)> = Vec::with_capacity(n_pads * (n_pads - 1) / 2);
            for i in 0..n_pads {
                for j in (i + 1)..n_pads {
                    edges.push((pads[i].distance_to(pads[j]), i, j));
                }
            }
            edges.sort_unstable_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            let mut dsu = DisjointSet::new(n_pads);
            let mut edges_count = 0;
            for (_dist, i, j) in edges {
                if dsu.union(i, j) {
                    ratsnest_lines.push(RatsnestLine::new(pads[i], pads[j], net_id));
                    edges_count += 1;
                    if edges_count == n_pads - 1 {
                        break;
                    }
                }
            }
            continue;
        }

        // 1. Cluster pads into copper-connected islands based on routed tracks and vias
        let tracks_of_net = tracks_by_net.get(&net_id);
        let vias_of_net = vias_by_net.get(&net_id);

        let mut dsu = DisjointSet::new(n_pads);

        if tracks_of_net.is_some() || vias_of_net.is_some() {
            // Determine connectivity among pads via copper tracks and vias
            let mut copper_nodes: Vec<Point> = Vec::new();
            if let Some(tracks) = tracks_of_net {
                for t in tracks {
                    copper_nodes.push(t.start);
                    copper_nodes.push(t.end);
                }
            }
            if let Some(vias) = vias_of_net {
                for v in vias {
                    copper_nodes.push(v.at);
                }
            }

            let mut copper_dsu = DisjointSet::new(copper_nodes.len());
            // Connect track endpoints
            if let Some(tracks) = tracks_of_net {
                for (t_idx, _t) in tracks.iter().enumerate() {
                    let start_idx = t_idx * 2;
                    let end_idx = t_idx * 2 + 1;
                    copper_dsu.union(start_idx, end_idx);
                }
            }

            // Connect copper nodes that are coincident or connected through vias
            for i in 0..copper_nodes.len() {
                let ci = copper_nodes[i];
                for j in (i + 1)..copper_nodes.len() {
                    let cj = copper_nodes[j];
                    if (ci.x - cj.x).abs() < 1e-3 && (ci.y - cj.y).abs() < 1e-3 {
                        copper_dsu.union(i, j);
                    }
                }
            }

            // Union pads that share any copper root directly without allocating HashSets
            let mut copper_root_first_pad: Vec<Option<usize>> = vec![None; copper_nodes.len()];
            for (p_idx, &pt) in pads.iter().enumerate() {
                for (c_idx, &c_pt) in copper_nodes.iter().enumerate() {
                    if (pt.x - c_pt.x).abs() < 1e-3 && (pt.y - c_pt.y).abs() < 1e-3 {
                        let root = copper_dsu.find(c_idx);
                        if let Some(first_pad) = copper_root_first_pad[root] {
                            dsu.union(first_pad, p_idx);
                        } else {
                            copper_root_first_pad[root] = Some(p_idx);
                        }
                    }
                }
            }
        }

        // 2. Identify distinct island components
        let mut island_map: HashMap<usize, Vec<usize>> = HashMap::new();
        for i in 0..n_pads {
            let root = dsu.find(i);
            island_map.entry(root).or_default().push(i);
        }

        let island_roots: Vec<usize> = island_map.keys().copied().collect();
        let n_islands = island_roots.len();
        if n_islands <= 1 {
            // All pads are connected in a single copper island -> no ratsnest line!
            continue;
        }

        // 3. Kruskal's algorithm across islands using minimum Euclidean distance
        struct IslandEdge {
            u: usize, // island index in island_roots
            v: usize, // island index in island_roots
            weight: f64,
            start_pad: Point,
            end_pad: Point,
        }

        let mut candidate_edges = Vec::new();
        for i in 0..n_islands {
            let root_i = island_roots[i];
            let pads_i = &island_map[&root_i];

            for j in (i + 1)..n_islands {
                let root_j = island_roots[j];
                let pads_j = &island_map[&root_j];

                let mut min_dist = f64::INFINITY;
                let mut best_pair = (Point::zero(), Point::zero());

                for &pi in pads_i {
                    let pt_i = pads[pi];
                    for &pj in pads_j {
                        let pt_j = pads[pj];
                        let dist = pt_i.distance_to(pt_j);
                        if dist < min_dist {
                            min_dist = dist;
                            best_pair = (pt_i, pt_j);
                        }
                    }
                }

                candidate_edges.push(IslandEdge {
                    u: i,
                    v: j,
                    weight: min_dist,
                    start_pad: best_pair.0,
                    end_pad: best_pair.1,
                });
            }
        }

        candidate_edges.sort_by(|a, b| a.weight.partial_cmp(&b.weight).unwrap_or(std::cmp::Ordering::Equal));

        let mut island_dsu = DisjointSet::new(n_islands);
        let mut edges_selected = 0;

        for edge in candidate_edges {
            if island_dsu.union(edge.u, edge.v) {
                ratsnest_lines.push(RatsnestLine::new(edge.start_pad, edge.end_pad, net_id));
                edges_selected += 1;
                if edges_selected == n_islands - 1 {
                    break;
                }
            }
        }
    }

    ratsnest_lines
}

// ============================================================================
// In-Module Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_arithmetic_and_distance() {
        let p1 = Point::new(1.0, 2.0);
        let p2 = Point::new(4.0, 6.0);
        assert_eq!(p1 + p2, Point::new(5.0, 8.0));
        assert_eq!(p2 - p1, Point::new(3.0, 4.0));
        assert_eq!(p1.distance_to(p2), 5.0);
        assert_eq!(p1.distance_to(p1), 0.0);
        assert_eq!(Point::new(0.0, 10.0).length(), 10.0);
        assert_eq!(p1.scale(2.0), Point::new(2.0, 4.0));
    }

    #[test]
    fn test_sin_cos_deg_exact_orthogonals() {
        assert_eq!(sin_cos_deg(0.0), (0.0, 1.0));
        assert_eq!(sin_cos_deg(90.0), (1.0, 0.0));
        assert_eq!(sin_cos_deg(180.0), (0.0, -1.0));
        assert_eq!(sin_cos_deg(270.0), (-1.0, 0.0));
        assert_eq!(sin_cos_deg(360.0), (0.0, 1.0));
        assert_eq!(sin_cos_deg(-90.0), (-1.0, 0.0));
        assert_eq!(sin_cos_deg(450.0), (1.0, 0.0));
    }

    #[test]
    fn test_point_rotation_zero_drift() {
        let original = Point::new(3.14159, 2.71828);
        let mut p = original;
        for _ in 0..4 {
            p = p.rotate_around(Point::zero(), 90.0);
        }
        assert_eq!(p, original);

        let p_rot = Point::new(1.0, 0.0).rotate_around(Point::zero(), 90.0);
        assert_eq!(p_rot, Point::new(0.0, 1.0));

        let center = Point::new(5.0, 5.0);
        let p_ctr = Point::new(6.0, 5.0).rotate_around(center, 180.0);
        assert_eq!(p_ctr, Point::new(4.0, 5.0));
    }

    #[test]
    fn test_rect_containment_and_intersection() {
        let r1 = Rect::from_coords(0.0, 0.0, 10.0, 10.0);
        let r2 = Rect::from_coords(5.0, 5.0, 15.0, 15.0);
        let r3 = Rect::from_coords(12.0, 12.0, 20.0, 20.0);

        assert!(r1.contains_point(Point::new(5.0, 5.0)));
        assert!(r1.contains_point(Point::new(0.0, 0.0)));
        assert!(!r1.contains_point(Point::new(11.0, 5.0)));

        assert!(r1.intersects(&r2));
        assert!(!r1.intersects(&r3));
        assert!(r2.intersects(&r3));
    }

    #[test]
    fn test_pad_shapes_bounding_box() {
        let center = Point::new(10.0, 20.0);

        let circle = PadShape::Circle { diameter: 2.0 };
        let bb_c0 = circle.bounding_box(center, 0.0);
        let bb_c45 = circle.bounding_box(center, 45.0);
        assert_eq!(bb_c0, Rect::from_coords(9.0, 19.0, 11.0, 21.0));
        assert_eq!(bb_c45, Rect::from_coords(9.0, 19.0, 11.0, 21.0));

        let rect = PadShape::Rect { width: 4.0, height: 2.0 };
        let bb_r0 = rect.bounding_box(center, 0.0);
        assert_eq!(bb_r0, Rect::from_coords(8.0, 19.0, 12.0, 21.0));

        let bb_r90 = rect.bounding_box(center, 90.0);
        assert_eq!(bb_r90, Rect::from_coords(9.0, 18.0, 11.0, 22.0));

        let obround = PadShape::Obround { width: 4.0, height: 2.0 };
        let bb_o0 = obround.bounding_box(center, 0.0);
        assert_eq!(bb_o0, Rect::from_coords(8.0, 19.0, 12.0, 21.0));

        let bb_o90 = obround.bounding_box(center, 90.0);
        assert_eq!(bb_o90, Rect::from_coords(9.0, 18.0, 11.0, 22.0));
    }

    #[test]
    fn test_netlist_ingestion_and_assignment() {
        let mut board = PcbBoard::new(50.0, 50.0);
        let fp1 = Footprint {
            reference: "U1".to_string(),
            value: "MCU".to_string(),
            package: "TQFP-32".to_string(),
            position: Point::new(20.0, 20.0),
            rotation: 0.0,
            pads: vec![
                Pad {
                    pin_number: "1".to_string(),
                    pin_name: "PD3".to_string(),
                    relative_pos: Point::new(-2.0, 0.0),
                    shape: PadShape::Rect { width: 1.0, height: 0.5 },
                    pad_type: PadType::Smd { layer: Layer::FCu },
                    net: None,
                },
                Pad {
                    pin_number: "2".to_string(),
                    pin_name: "PD4".to_string(),
                    relative_pos: Point::new(-2.0, 0.8),
                    shape: PadShape::Rect { width: 1.0, height: 0.5 },
                    pad_type: PadType::Smd { layer: Layer::FCu },
                    net: None,
                },
            ],
            silkscreen_lines: vec![],
        };
        board.footprints.push(fp1);

        assert!(board.assign_pad_net("U1", "1", "GND").is_ok());
        let gnd_id = board.get_or_create_net_id("GND");
        assert_eq!(board.footprints[0].pads[0].net, Some(gnd_id));
        assert_eq!(board.get_net_name(gnd_id), Some("GND"));

        let err = board.assign_pad_net("U1", "1", "+5V").unwrap_err();
        match err {
            IngestionError::DuplicatePinAssignment {
                component_ref,
                pin_id,
                existing_net_name,
                new_net_name,
            } => {
                assert_eq!(component_ref, "U1");
                assert_eq!(pin_id, "1");
                assert_eq!(existing_net_name, "GND");
                assert_eq!(new_net_name, "+5V");
            }
            _ => panic!("Expected DuplicatePinAssignment error"),
        }

        assert!(matches!(
            board.assign_pad_net("U99", "1", "GND"),
            Err(IngestionError::ComponentNotFound { .. })
        ));

        assert!(matches!(
            board.assign_pad_net("U1", "99", "GND"),
            Err(IngestionError::PinNotFound { .. })
        ));
    }

    #[test]
    fn test_board_validation_pass() {
        let mut board = PcbBoard::new(50.0, 50.0);
        let fp = Footprint {
            reference: "R1".to_string(),
            value: "10k".to_string(),
            package: "0805".to_string(),
            position: Point::new(25.0, 25.0),
            rotation: 0.0,
            pads: vec![
                Pad {
                    pin_number: "1".to_string(),
                    pin_name: "1".to_string(),
                    relative_pos: Point::new(-0.95, 0.0),
                    shape: PadShape::Rect { width: 1.0, height: 1.3 },
                    pad_type: PadType::Smd { layer: Layer::FCu },
                    net: None,
                },
                Pad {
                    pin_number: "2".to_string(),
                    pin_name: "2".to_string(),
                    relative_pos: Point::new(0.95, 0.0),
                    shape: PadShape::Rect { width: 1.0, height: 1.3 },
                    pad_type: PadType::Smd { layer: Layer::FCu },
                    net: None,
                },
            ],
            silkscreen_lines: vec![],
        };
        board.footprints.push(fp);

        let report = board.validate();
        assert!(report.is_valid());
        assert_eq!(report.total_footprints, 1);
        assert_eq!(report.total_pads, 2);
        assert_eq!(report.unconnected_pads, 2);
    }
}
