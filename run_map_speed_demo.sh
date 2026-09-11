#!/usr/bin/env bash
set -euo pipefail
# Simulator-only opt-in. No claim of validated tracking at this speed.
export TRIAL_HIGH_SPEED_KPH="${1:-60}"
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$workspace/run_team_demo.sh"
