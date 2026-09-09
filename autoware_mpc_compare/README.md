# Autoware AI MPC comparison — opt-in, ROS1

This is the **archived official ROS1 Autoware AI** MPC, not current ROS2
Autoware Universe. Controller sources under mpc_follower/src and include are
unchanged. Original Apache notices and qpOASES LGPL source/license are retained.
See the validation artifact provenance.json for pinned commits and source hashes.

Only build manifests were adapted for this existing Noetic installation.
autoware_msgs builds the original eight message definitions needed by MPC;
it is not a complete Autoware installation. Source this overlay only for the
comparison workflow, not for unrelated Autoware nodes.

## Data and command flow

Existing RDDF-selected, boundary-centered route + GPS/IMU localization
→ ROS adapter (same route XY, 4 m of rear context, measured velocity)
→ official mpc_follower + qpOASES
→ adapter (freshness checks, ±40° angle / 60°/s rate limits)
→ existing MORAI sender.

Existing controller runs separately for longitudinal MPC and its unchanged
fixed-corridor/heading guard formulas. A new default-OFF longitudinal_only
option skips its unused steering optimizer, using direct route projection for
heading-guard geometry. The original spatial steering-path fit is not used
in this comparison's heading-guard geometry. Its zero steering placeholder on
/comparison/custom_command is NOT sent to the simulator.
/control/actuator_command and /control/controller_status contain the combined
actual command. Global raw-GPS projection, not internal filtered CTE alone,
is used to compare tracking.

No intentional inward path offset. No XY smoothing in the official MPC.
No measured front-wheel steering angle is available in Competition Status.
Initial tests use official kinematics_no_delay with pure input-delay
compensation. The diagnostic effective steering derived from IMU yaw rate
is NOT a measured tire angle and is not used by this two-state model.
This model does not represent a complete first-order steering actuator.

The original three-state lag model is preserved but excluded initially:
its discretized steady-state steering gain is 1.33333 for DT=.1, tau=.15.
The standalone check_official_model.cpp reproduces this independently.

## Safe launch

Source the main workspace install/setup.bash, then this overlay install/setup.bash.
roslaunch morai_autoware_compare compare.launch send_control:=false

Actual simulator actuation is explicitly opt-in, send_control:=true.
Before trials: Manual → I → P, verify initial coordinates, stationary and fresh
inputs. The saved run_trial.py performs these checks and stops its processes.
Do not run alongside another sender/controller publishing the same outputs.

## Rollback

The original control gains, controller library and default launch are unchanged.
The main controller node adds a default-OFF longitudinal_only option, explicitly
false in its main YAML; its original source and binary are backed up.
Stop this comparison and use a fresh terminal sourcing only the main workspace.
No simulator performance improvement is claimed by successful build/unit tests.
