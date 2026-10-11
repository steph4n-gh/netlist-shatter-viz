#!/bin/sh
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
test -x .venv/bin/python || { echo 'Run ./scripts/install-alpha.sh first.' >&2; exit 1; }
.venv/bin/python scripts/doctor.py --require-routing
# Native routing must use the same Java executable checked by doctor.
JAVA="${JAVA:-$(command -v java)}"
export JAVA
exec .venv/bin/python scripts/layout_server.py "$@"
