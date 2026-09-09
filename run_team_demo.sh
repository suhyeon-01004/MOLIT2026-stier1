#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /home/stier/molit-2026-stier1-suhyeon/install/setup.bash
source /home/stier/molit-2026-stier1-suhyeon/autoware_mpc_compare/install/setup.bash
demo_dir=/home/stier/molit-2026-stier1-suhyeon/artifacts/controller_validation/2026-09-09/team_demo
export DISPLAY="${DISPLAY:-:0}"
# Stop at lap completion; 260 seconds is the maximum driving time.
export TRIAL_SECONDS=260
export TRIAL_RVIZ=1
export TRIAL_AUTOWARE_CONFIG="$demo_dir/mpc_demo.yaml"
export TRIAL_LOCALIZATION_CONFIG="$demo_dir/localization.yaml"
unset TRIAL_EXCITATION TRIAL_MAP_GUARD
exec /usr/bin/python3 "$demo_dir/run_trial.py" "team_demo_$(date +%Y%m%d_%H%M%S)" "$demo_dir/controller_baseline.yaml"
