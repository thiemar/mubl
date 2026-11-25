#!/bin/sh
# Regenerate src/dronecan/{classic,fd}/dronecan.[ch] with sfwa-dronecan's bootloader profile.
#
#   tools/gen_dronecan.sh [SFWA_DRONECAN_DIR] [DSDL_DIR]
#
# Defaults: ../dronecan next to this repository, and the standard DSDL its conformance run fetched.
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
dc=${1:-$here/../dronecan}
dsdl=${2:-$dc/conformance/build/upstream/DSDL/uavcan}
[ -d "$dsdl" ] || { echo "no DSDL at $dsdl (run python3 conformance/run.py in sfwa-dronecan)" >&2; exit 1; }

for frame in classic fd; do
    PYTHONPATH="$dc/python" python3 -m sfwa_dronecan generate -o "$here/src/dronecan/$frame" \
        --profile bootloader --frame $frame \
        -t uavcan.protocol.NodeStatus=encode \
        -t uavcan.protocol.GetNodeInfo=request:decode,response:encode \
        -t uavcan.protocol.dynamic_node_id.Allocation=encode+view \
        -t uavcan.protocol.file.BeginFirmwareUpdate=request:view \
        -t uavcan.protocol.file.Read=request:encode,response:view \
        -t uavcan.protocol.debug.LogMessage=encode \
        -t uavcan.protocol.RestartNode=request:decode,response:encode \
        "$dsdl"
done
