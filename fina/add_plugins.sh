#!/bin/bash

paths+=($(lsautoware/src/core/autoware_core/planning/behavior_path_planner))
paths+=($(ls autoware/src/core/autoware_core/planning/behavior_velocity_planner))
paths+=($(ls autoware/src/core/autoware_core/planning/motion_velocity_planner))
paths+=($(ls autoware/src/universe/autoware_universe/planning/behavior_path_planner))
paths+=($(ls autoware/src/universe/autoware_universe/planning/behavior_velocity_planner))
paths+=($(ls autoware/src/universe/autoware_universe/planning/motion_velocity_planner))

for i in "${!paths[@]}"; do
    cat >> plugin_pkgs.txt << EOF
${paths[$i]}
EOF
done