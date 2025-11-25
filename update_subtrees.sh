#!/bin/bash

echo "🔄 Updating all subtree repos inside marinero_stack..."

# Define all repos to update (prefix → repo name)
declare -A repos=(
    [goal_assignment]="goal_assignment"
    [mapviz]="mapviz"
    [marinero_control]="marinero_control"
    [marinero_navigation]="marinero_navigation"
    [marinero_pointclouds]="marinero_pointclouds"
    [marinero_simulations]="marinero_simulations"
    [robot_localization]="robot_localization"
)

# Loop through and update each repo
for prefix in "${!repos[@]}"; do
    repo_name="${repos[$prefix]}"
    repo_url="https://github.com/albic98/${repo_name}.git"

    echo "---------------------------------------------"
    echo "⏳ Pulling updates for: $repo_name → $prefix/"
    echo "From: $repo_url"
    echo "---------------------------------------------"

    if [[ "$repo_name" == "mapviz" ]]; then
        echo "⚠️ Special handling for mapviz: using branch 'ros2-devel'"
        git subtree pull --prefix="$prefix" "$repo_url" ros2-devel --squash
    else
        git subtree pull --prefix="$prefix" "$repo_url" main --squash
    fi

    if [ $? -ne 0 ]; then
        echo "❌ Failed to update $prefix"
    else
        echo "✅ Successfully updated $prefix"
    fi
done

echo "---------------------------------------------"
echo "🚀 Pushing updates to marinero_stack..."
git push
echo "✅ All done!"