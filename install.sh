#!/usr/bin/env bash
# Install the CRSFRcModule overlay into a Meshtastic firmware checkout.
#
#   ./install.sh /path/to/meshtastic            # firmware dir (contains src/modules/Modules.cpp)
#
# Idempotent: safe to re-run. Copies the module sources and registers the
# module in setupModules() by inserting two lines (only if not already present).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="${1:-}"

if [[ -z "$FW" ]]; then
  echo "Usage: ./install.sh /path/to/meshtastic"
  echo "  (the firmware directory that contains src/modules/Modules.cpp)"
  exit 2
fi

MODULES="$FW/src/modules/Modules.cpp"
if [[ ! -f "$MODULES" ]]; then
  echo "ERROR: $MODULES not found — '$FW' does not look like a Meshtastic firmware checkout." >&2
  exit 1
fi

echo "==> Copying module sources into $FW/src/modules/"
mkdir -p "$FW/src/modules"
cp -v "$HERE/src/modules/CRSFRcModule.h"   "$FW/src/modules/CRSFRcModule.h"
cp -v "$HERE/src/modules/CRSFRcModule.cpp" "$FW/src/modules/CRSFRcModule.cpp"

echo "==> Registering module in setupModules() (idempotent)"
python3 - "$MODULES" <<'PY'
import sys, re
path = sys.argv[1]
src = open(path, encoding="utf-8").read()
changed = False

# 1) header include — place after the StatusMessageModule include block
if '#include "modules/CRSFRcModule.h"' not in src:
    anchor = '#include "modules/StatusMessageModule.h"'
    idx = src.find(anchor)
    if idx != -1:
        endif = src.find("#endif", idx)
        nl = src.find("\n", endif if endif != -1 else idx)
        src = src[:nl] + '\n\n#include "modules/CRSFRcModule.h"' + src[nl:]
    else:
        src = '#include "modules/CRSFRcModule.h"\n' + src
    changed = True

# 2) construction — must be created last; right after RoutingModule
if 'new CRSFRcModule();' not in src:
    m = re.search(r'(routingModule\s*=\s*new\s+RoutingModule\(\);[ \t]*\n)', src)
    if not m:
        raise SystemExit("Could not locate 'routingModule = new RoutingModule();'.\n"
                         "Add 'new CRSFRcModule();' manually at the end of setupModules().")
    src = src[:m.end()] + "\n    new CRSFRcModule();\n" + src[m.end():]
    changed = True

if changed:
    open(path, "w", encoding="utf-8").write(src)
    print("   Modules.cpp updated.")
else:
    print("   Already registered — nothing to do.")
PY

echo
echo "==> Done. Now build & flash:"
echo "    cd '$FW'"
echo "    pio run -e heltec-v3"
echo "    # flash firmware.bin from .pio/build/heltec-v3/ with esptool"
