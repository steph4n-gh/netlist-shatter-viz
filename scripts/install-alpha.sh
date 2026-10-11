#!/bin/sh
# Install into this checkout; no global packages or external EDA tools are installed.
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
command -v uv >/dev/null 2>&1 || { echo 'Install uv, then run this command again.' >&2; exit 1; }
command -v cargo >/dev/null 2>&1 || { echo 'Install Rust with rustup, then run this command again.' >&2; exit 1; }
uv venv --python 3.12.9 .venv
uv pip install --python .venv/bin/python --require-hashes --only-binary=:all: -r scripts/requirements-lock.txt
cargo build --release --bins --locked
.venv/bin/python scripts/doctor.py
echo 'Installed. Set JAVA and FREEROUTING_JAR for Java 25+ and Freerouting 2.4.1, then run ./scripts/start-alpha.sh.'
