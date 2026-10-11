//! Netlist Shatter PCB CAD, Autorouting, and CAM Export Suite.
//!
//! Provides core headless PCB layout, physical footprint modeling,
//! ratsnest computation, DRC verification, and KiCad/Gerber export pipelines.

pub mod pcb;

// Re-export pcb types at crate root for ergonomic integration in tests and external tools
pub use pcb::*;
