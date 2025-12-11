#!/bin/bash

if [[ "$0" == "$BASH_SOURCE" ]]; then
    echo "This script must be sourced. Use: . $0"
    exit 1
fi



. pop.sh

. plugin_build.sh

packages=()
while IFS= read -r line || [[ -n "$line" ]]; do
    [[ -n "$line" ]] && packages+=("$line")
done < pkgs.txt

cd autoware

. /opt/ros/humble/setup.bash

for i in "${!packages[@]}"; do
    echo "Building ${packages[$i]}"
    colcon build --packages-up-to ${packages[$i]}
done

cd -