# Third-party notices

The root MIT license covers original Netlist Shatter source code and documentation. Third-party material keeps its stated license and copyright notices; the root license does not relicense it. Product and manufacturer names identify sources and components and do not imply endorsement.

## Circuit and footprint material

- **Adafruit TCA9548A derivative:** Designed by Limor Fried/Ladyada for Adafruit Industries. Upstream: https://github.com/adafruit/Adafruit-TCA9548A-I2C-Multiplexer-PCB, commit `c5bdfe4c8e860cee7400539d6e45a235da7afcd2`. The upstream source in `reference/adafruit-tca9548a/` and exported circuit/layout derivatives are **CC-BY-SA-3.0**, not MIT. Keep the complete upstream README and license. Netlist Shatter replaces solder jumpers with 0805 0R/DNP links, omits artwork and original pours, and uses fresh placement/routing. See `licenses/adafruit-tca9548a/`.
- **KiCad community footprints:** Portions of the RP2040 bring-up example use KiCad footprint data, including `USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal`. KiCad library data is **CC-BY-SA-4.0 with the electronic-design exception**: https://www.kicad.org/libraries/license/. This exception waives article 3 for electronic designs and files generated from those designs; it does not apply to redistributing library collections. Preserve `licenses/kicad-library/LICENSE.md` if distributing extracted footprint data. GCT's drawing URL is a component reference; no GCT PDF or third-party 3D model is bundled.

## Rust dependencies

`licenses/rust/DEPENDENCIES.json` records all registry packages resolved by the alpha's Cargo.lock, their metadata-declared license expressions, upstream URLs, authors, and copied license texts. Where dual licensing is offered, MIT is used for this distribution, except `foldhash` (Zlib) and unicode-ident's additional Unicode-3.0 license. The copied license texts preserve original copyright notices. Some published crates omit a standalone license file; this inventory explicitly retains their Cargo metadata instead of claiming that a copied file exists.

Macroquad embeds **ProggyClean.ttf**, Copyright (c) 2004, 2005 Tristan Grimmer, under MIT. The full notice is in `licenses/ProggyClean/LICENSE`; upstream https://github.com/bluescan/proggyfonts. Fonts used only in dependency examples are not part of the alpha binaries.

## Separately installed software

Python packages, KiCad, Java, uv and Freerouting are installed separately and are not relicensed by this repository. The alpha source archive does not contain Python wheels, KiCad, a JRE or Freerouting binary.

- gerbonara 1.6.3: Apache-2.0, https://pypi.org/project/gerbonara/1.6.3/.
- Shapely 2.1.2: BSD-3-Clause. Its wheels include GEOS (LGPL-2.1); https://pypi.org/project/shapely/2.1.2/. Preserve wheel notices and GEOS source/relinking obligations when redistributing wheels.
- NumPy 2.5.3 and SciPy 1.18.1: BSD-based distributions with additional bundled third-party licenses. Installed wheels carry their complete notices; https://pypi.org/project/numpy/2.5.3/ and https://pypi.org/project/scipy/1.18.1/.
- KiCad: GPL software; its community libraries use the separate library license described above, https://www.kicad.org/about/licenses/.
- Freerouting: GPL-3.0. A separately distributed patched router must include its GPL text, exact corresponding patched source and build/installation materials. The local fanout patch adds a `ctrl.withNeckdown` guard to honor disabled neckdown. A source URL alone is not a substitute for conveying required corresponding source with a distributed binary.

## Excluded historical material

This alpha omits ModRetro downloads and recovered derivatives, vendor PDFs, imported full hardware repositories, historical manufacturing ZIPs, local job outputs, private experiments and bundled runtimes. Their presence in the development workspace does not mean that they are distributed under the root MIT license.
