#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$workspace/install/setup.bash"
demo_dir="$workspace/artifacts/controller_validation/2026-09-09/team_demo"
export DISPLAY="${DISPLAY:-:0}"
export TRIAL_HIGH_SPEED_KPH="${TRIAL_HIGH_SPEED_KPH:-60}"
export TRIAL_ROUTE_CANDIDATE="${TRIAL_ROUTE_CANDIDATE-$workspace/src/morai_kcity_hd_map/config/route_candidate_default.json}"
# Stop at lap completion; 260 seconds is the maximum driving time.
export TRIAL_SECONDS=260
export TRIAL_RVIZ=1
export TRIAL_AUTOWARE_CONFIG="$workspace/src/morai_path_tracking/config/controllers/autoware_mpc.yaml"
export TRIAL_LOCALIZATION_CONFIG="$workspace/src/morai_localization/config/maps/molit_2026_kcity.yaml"
unset TRIAL_EXCITATION TRIAL_MAP_GUARD
exec /usr/bin/python3 "$demo_dir/run_trial.py" "team_demo_$(date +%Y%m%d_%H%M%S)" "$workspace/src/morai_path_tracking/config/controllers/molit_2026_path_tracking.yaml"
