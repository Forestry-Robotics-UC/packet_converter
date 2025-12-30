#!/bin/bash
set -euo pipefail

# If a ROS/colcon install exists, source it so env vars are available
# Some setup scripts assume unset variables and will fail under 'set -u'.
# Temporarily disable nounset while sourcing them.
set +u
if [ -f /docker_ws/install/setup.bash ]; then
    echo "Sourcing /docker_ws/install/setup.bash"
    # shellcheck disable=SC1091
    source /docker_ws/install/setup.bash || true
fi
# Re-enable nounset now that sourcing is done
set -u

# Activate the virtualenv if present
if [ -f /app/.venv/bin/activate ]; then
    # shellcheck disable=SC1091
    source /app/.venv/bin/activate
fi

# If first arg starts with - or no args provided, forward as-is
# Run the converter script from workspace root
SCRIPT_PATH="/app/convert_bag_packets.py"
if [ ! -f "$SCRIPT_PATH" ]; then
    echo "ERROR: $SCRIPT_PATH not found"
    exec bash
fi

# If no args provided, keep default
if [ "$#" -eq 0 ]; then
    exec python "$SCRIPT_PATH"
else
    exec python "$SCRIPT_PATH" "$@"
fi
