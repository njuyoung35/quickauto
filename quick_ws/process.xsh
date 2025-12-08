#!/usr/bin/env xonsh

import yaml
import os

repos_path = "repos.yaml"
src_path = "src"

with open(repos_path, "r") as f:
    repos = yaml.safe_load(f)

for repo in repos["repos"]:
    repo_name = repo["repo"]
    subpaths = repo["subpaths"]
    
    if not os.path.exists(f"{src_path}/{repo_name}"):
        os.makedirs(f"{src_path}/{repo_name}")

    for subpath in subpaths:
        target_path = os.path.join(f"{src_path}/{repo_name}", subpath)

        if os.path.exists(target_path):
            print(f"[-] Skipping {target_path}")
            continue
        os.makedirs(target_path)

        source_path = os.path.join(f"../{repo_name}", subpath)

        cp @(source_path) @(target_path) -r