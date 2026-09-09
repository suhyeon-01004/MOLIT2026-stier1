#!/usr/bin/env bash
set -euo pipefail
# Simulator-only opt-in. No claim of validated tracking at this speed.
export TRIAL_HIGH_SPEED_KPH="${1:-100}"
exec bash /home/stier/molit-2026-stier1-suhyeon/run_team_demo.sh
