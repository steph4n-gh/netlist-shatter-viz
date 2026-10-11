//! Assembly, Manufacturing & SMT Deliverables (BOM, CPL, ZIP Archive)
//!
//! Provides utilities to generate:
//! - Bill of Materials (`BOM.csv`) for SMT component sourcing and kitting
//! - Pick-and-Place Centroid table (`CPL.csv`) for automated SMT placement
//! - Engineering ZIP archives (`.zip`) containing CAM Gerber & Excellon drill files
//!
//! Packaging alone does not qualify parts, electrical operation or manufacturing release.

use crate::pcb::core::PcbBoard;
use serde::{Deserialize, Serialize};

/// Canonical Bill of Materials (BOM) row item adhering to PCBWay / JLCPCB SMT specifications.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BomEntry {
    pub item: usize,
    pub designator: String,
    pub value: String,
    pub package: String,
    pub quantity: usize,
    pub description: String,
    pub manufacturer: String,
    pub mpn: String,
}

impl BomEntry {
    pub fn new(
        item: usize,
        designator: impl Into<String>,
        value: impl Into<String>,
        package: impl Into<String>,
        quantity: usize,
        description: impl Into<String>,
        manufacturer: impl Into<String>,
        mpn: impl Into<String>,
    ) -> Self {
        Self {
            item,
            designator: designator.into(),
            value: value.into(),
            package: package.into(),
            quantity,
            description: description.into(),
            manufacturer: manufacturer.into(),
            mpn: mpn.into(),
        }
    }
}

/// Centroid Pick-and-Place (CPL) entry specifying component placement coordinates for SMT feeders.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CplEntry {
    pub designator: String,
    pub mid_x: f64,
    pub mid_y: f64,
    pub layer: String,
    pub rotation: f64,
}

impl CplEntry {
    pub fn new(
        designator: impl Into<String>,
        mid_x: f64,
        mid_y: f64,
        layer: impl Into<String>,
        rotation: f64,
    ) -> Self {
        Self {
            designator: designator.into(),
            mid_x: (mid_x * 10000.0).round() / 10000.0,
            mid_y: (mid_y * 10000.0).round() / 10000.0,
            layer: layer.into(),
            rotation: ((rotation % 360.0) + 360.0) % 360.0,
        }
    }
}

/// Exports a Bill of Materials table into a standardized CSV string.
pub fn export_bom_csv(entries: &[BomEntry]) -> String {
    let mut out = String::with_capacity(4096);
    out.push_str("Item,Designator,Value,Package,Quantity,Description,Manufacturer,MPN\n");
    for e in entries {
        out.push_str(&format!(
            "{},\"{}\",\"{}\",\"{}\",{},\"{}\",\"{}\",\"{}\"\n",
            e.item, e.designator, e.value, e.package, e.quantity, e.description, e.manufacturer, e.mpn
        ));
    }
    out
}

/// Lists actual board parts without implying that their land patterns or sourcing are qualified.
pub fn generate_bom_from_board(board: &PcbBoard) -> Vec<BomEntry> {
    let ap2112 = crate::pcb::footprint::ap2112k_33("");
    board.footprints.iter().enumerate().map(|(i, f)| {
        let mut row = BomEntry::new(i + 1, &f.reference, &f.value, &f.package, 1,
            "Engineering BOM: sourcing/land pattern unqualified", "", "");
        // Supply the reviewed exact part only while its actual lands still match.
        // This does not qualify capacitor bias, board loads, thermal or hardware.
        if f.value == ap2112.value && f.package == ap2112.package
            && f.pads.len() == ap2112.pads.len()
            && f.pads.iter().zip(&ap2112.pads).all(|(pad, reference)| {
                pad.pin_number == reference.pin_number
                    && pad.relative_pos == reference.relative_pos
                    && pad.shape == reference.shape && pad.pad_type == reference.pad_type
            })
        {
            row.manufacturer = "Diodes Incorporated".into();
            row.mpn = ap2112.value.clone();
            row.description = "Engineering BOM: exact AP2112 part and reference lands checked; load, capacitor, thermal and hardware unqualified".into();
        }
        row
    }).collect()
}

/// Exports a Pick-and-Place Centroid table into a standardized CSV string.
pub fn export_cpl_csv(entries: &[CplEntry]) -> String {
    let mut out = String::with_capacity(4096);
    out.push_str("Designator,Mid X,Mid Y,Layer,Rotation\n");
    for e in entries {
        out.push_str(&format!(
            "{},{:.4},{:.4},{},{:.1}\n",
            e.designator, e.mid_x, e.mid_y, e.layer, e.rotation
        ));
    }
    out
}

/// Automatically extracts Pick-and-Place Centroid entries from all footprints on a `PcbBoard`.
pub fn generate_cpl_from_board(board: &PcbBoard) -> Vec<CplEntry> {
    let mut entries = Vec::with_capacity(board.footprints.len());
    for fp in &board.footprints {
        entries.push(CplEntry::new(
            &fp.reference,
            fp.position.x,
            fp.position.y,
            if fp.pads.iter().any(|p|matches!(p.pad_type,crate::pcb::core::PadType::Smd{layer:crate::pcb::core::Layer::BCu}))
                && !fp.pads.iter().any(|p|matches!(p.pad_type,crate::pcb::core::PadType::Smd{layer:crate::pcb::core::Layer::FCu})) {"Bottom"}else{"Top"},
            fp.rotation,
        ));
    }
    // Sort deterministically by designator
    entries.sort_by(|a, b| a.designator.cmp(&b.designator));
    entries
}

// ============================================================================
// Lightweight Pure-Rust ZIP Archive Generator
// ============================================================================

/// Computes the standard IEEE 802.3 CRC-32 checksum for a slice of bytes.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = -(crc as i32 & 1) as u32;
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// Bundles a collection of named files into a standard, valid ZIP archive buffer (stored/uncompressed).
///
/// Fully compatible with standard unzippers, Python `zipfile`, PCBWay, JLCPCB, and OSH Park.
pub fn create_zip_archive(files: &[(&str, &[u8])]) -> Vec<u8> {
    let mut archive = Vec::new();
    let mut central_directory = Vec::new();

    for &(filename, data) in files {
        let name_bytes = filename.as_bytes();
        let name_len = name_bytes.len() as u16;
        let data_len = data.len() as u32;
        let checksum = crc32(data);
        let local_header_offset = archive.len() as u32;

        // --- Local File Header ---
        archive.extend_from_slice(&0x04034b50u32.to_le_bytes()); // Local file header signature
        archive.extend_from_slice(&20u16.to_le_bytes());         // Version needed: 2.0
        archive.extend_from_slice(&0u16.to_le_bytes());          // General purpose bit flag
        archive.extend_from_slice(&0u16.to_le_bytes());          // Compression method: 0 (Store)
        archive.extend_from_slice(&0x4e21u16.to_le_bytes());      // Last mod file time
        archive.extend_from_slice(&0x553cu16.to_le_bytes());      // Last mod file date
        archive.extend_from_slice(&checksum.to_le_bytes());      // CRC-32
        archive.extend_from_slice(&data_len.to_le_bytes());      // Compressed size
        archive.extend_from_slice(&data_len.to_le_bytes());      // Uncompressed size
        archive.extend_from_slice(&name_len.to_le_bytes());      // File name length
        archive.extend_from_slice(&0u16.to_le_bytes());          // Extra field length
        archive.extend_from_slice(name_bytes);                   // File name
        archive.extend_from_slice(data);                         // File data

        // --- Central Directory Header ---
        central_directory.extend_from_slice(&0x02014b50u32.to_le_bytes()); // Central directory signature
        central_directory.extend_from_slice(&20u16.to_le_bytes());         // Version made by: 2.0
        central_directory.extend_from_slice(&20u16.to_le_bytes());         // Version needed: 2.0
        central_directory.extend_from_slice(&0u16.to_le_bytes());          // General purpose bit flag
        central_directory.extend_from_slice(&0u16.to_le_bytes());          // Compression method: 0 (Store)
        central_directory.extend_from_slice(&0x4e21u16.to_le_bytes());      // Last mod file time
        central_directory.extend_from_slice(&0x553cu16.to_le_bytes());      // Last mod file date
        central_directory.extend_from_slice(&checksum.to_le_bytes());      // CRC-32
        central_directory.extend_from_slice(&data_len.to_le_bytes());      // Compressed size
        central_directory.extend_from_slice(&data_len.to_le_bytes());      // Uncompressed size
        central_directory.extend_from_slice(&name_len.to_le_bytes());      // File name length
        central_directory.extend_from_slice(&0u16.to_le_bytes());          // Extra field length
        central_directory.extend_from_slice(&0u16.to_le_bytes());          // File comment length
        central_directory.extend_from_slice(&0u16.to_le_bytes());          // Disk number start
        central_directory.extend_from_slice(&0u16.to_le_bytes());          // Internal file attributes
        central_directory.extend_from_slice(&0u32.to_le_bytes());          // External file attributes
        central_directory.extend_from_slice(&local_header_offset.to_le_bytes()); // Relative offset of local header
        central_directory.extend_from_slice(name_bytes);                   // File name
    }

    let cd_offset = archive.len() as u32;
    let cd_size = central_directory.len() as u32;
    let file_count = files.len() as u16;

    // Append Central Directory to Archive
    archive.extend_from_slice(&central_directory);

    // --- End of Central Directory Record (EOCD) ---
    archive.extend_from_slice(&0x06054b50u32.to_le_bytes()); // EOCD signature
    archive.extend_from_slice(&0u16.to_le_bytes());          // Number of this disk
    archive.extend_from_slice(&0u16.to_le_bytes());          // Disk where central directory starts
    archive.extend_from_slice(&file_count.to_le_bytes());    // Number of central directory records on this disk
    archive.extend_from_slice(&file_count.to_le_bytes());    // Total number of central directory records
    archive.extend_from_slice(&cd_size.to_le_bytes());       // Size of central directory
    archive.extend_from_slice(&cd_offset.to_le_bytes());     // Offset of start of central directory
    archive.extend_from_slice(&0u16.to_le_bytes());          // Comment length

    archive
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc32_known_vector() {
        assert_eq!(crc32(b"123456789"), 0xCBF43926);
    }

    #[test]
    fn test_zip_archive_structure() {
        let files = [
            ("hello.txt", b"Hello, World!" as &[u8]),
            ("test.csv", b"a,b,c\n1,2,3" as &[u8]),
        ];
        let zip = create_zip_archive(&files);
        assert!(!zip.is_empty());
        assert_eq!(&zip[0..4], &0x04034b50u32.to_le_bytes());
    }
}
