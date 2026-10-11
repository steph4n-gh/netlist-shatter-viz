# v0.1.0-alpha.2

Automatic native **Finish** now discovers bounded cuts in obstructing copper
and restores the affected nets together. It includes the completion work for
`RGB_DATA` and `I2C_SCL` on the retained original CORE03 native Pico upload,
without a prepared cut-board input or manual trace coordinates.

The acceptance gates preserve the first source's complete contact obligations,
exact surviving copper, matching project/profile bytes, protected copper and
supplied routing limits. An incomplete selected cluster cannot be accepted as
a completed restoration. The release's `VERIFICATION.json` binds the exact
original upload, ordinary application result and clean-install replay to the
released source; it records the executed native/source/profile/export checks.
`SOURCE-MANIFEST.sha256` and `SHA256SUMS` identify the published files.

This is native partial-board completion for the recorded input. It does not
establish cold routing of the built-in Pico model, universal routing success,
physical operation or manufacturing release. The development upload, private
experiments and preselected manufacturing archives are not bundled. Licensing
and the external stock Freerouting/KiCad tool policy remain unchanged.

See [Quickstart](QUICKSTART.md), [Limitations](LIMITATIONS.md) and
[Testing](TESTING.md). The alpha 1 notes below retain the earlier release's
scope and incomplete Pico status.

# v0.1.0-alpha.1

First public source alpha of the local Netlist Shatter PCB workbench.

The release provides bounded placement experiments, native KiCad creation and
routing/repair, guarded improvement, local browser review and conditional
engineering-file exports. It includes runnable examples and tests; it excludes
historical job caches, private routing policy variants and preselected
manufacturing archives.

**Known limitation: Pico Ultimate is incomplete. `RGB_DATA` and `I2C_SCL` remain
open; no completed Pico handoff is provided.** Isolated candidate path checks
do not establish simultaneous routing or native board acceptance. No included
board has been physically validated or approved for manufacturing release.

See [Quickstart](QUICKSTART.md) for installation and [Limitations](LIMITATIONS.md)
for the checked scope. The release attaches `VERIFICATION.json` recording the
exact tagged commit, source archive SHA-256, test results, platform/tool versions
and clean-install/native-routing checks. `SHA256SUMS` covers the uploaded assets.

The supported alpha host is macOS on Apple silicon. Install and launch scripts
use pinned Python packages, a locked Rust build and separately installed KiCad
10.x, Java 25+ and the exact stock Freerouting 2.4.1 JAR. Historical results using
the private patched router do not qualify this package.
