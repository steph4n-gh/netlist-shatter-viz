pub mod assembly;
pub mod core;
pub mod drc;
pub mod drill;
pub mod esp32_cam_pro;
pub mod footprint;
pub mod framework_card;
pub mod gerber;
pub mod kicad;
pub mod m64_cart_pro;
pub mod pico_ultimate;
pub mod reference;
pub mod router;
pub mod rp2040;
pub mod specctra;

// Re-export core geometric primitives, netlist ingestion, and validation models
pub use self::core::*;

// Re-export parametric footprint generators and layout utilities
pub use self::footprint::*;

// Re-export automated 2-layer router and DRC engine
pub use self::drc::*;
pub use self::router::*;

// Re-export CAM exporters (KiCad, RS-274X Gerber, Excellon NC drill, Specctra DSN/SES)
pub use self::drill::*;
pub use self::gerber::*;
pub use self::kicad::*;
pub use self::specctra::*;

// Re-export Microcontroller Reference Systems & Production Designs
pub use self::assembly::*;
pub use self::esp32_cam_pro::*;
pub use self::framework_card::*;
pub use self::m64_cart_pro::*;
pub use self::pico_ultimate::*;
pub use self::reference::*;


pub mod bringup;
pub use bringup::*;

pub mod electrical;
pub use electrical::*;

pub mod pipeline;
pub use pipeline::*;

pub mod placement;
pub use placement::*;
pub mod route_cleanup;
pub use route_cleanup::*;
