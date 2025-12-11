#!/usr/bin/env xonsh

import yaml
from jinja2 import Template, Environment
import os
import sys
sys.path.append(".")
from universe import *
from meta_config import MetaConfig
from generate_vehicles import *

from typing import Dict
import re
import tempfile
import shutil

env = Environment(
    trim_blocks=True,
    lstrip_blocks=True,
    keep_trailing_newline=False
)
def comment_hash(text):
    return comment_text(text, start='# ')
def comment_slash(text):
    return comment_text(text, start='// ')
def comment_xml(text):
    return comment_text(text, start='<!-- ', end=' -->')
def comment_text(text, start='# ', end=' '):
    return '\n'.join(start + line + end for line in text.strip().split('\n')).strip()
def indent(text, num=4, indent_first_line=False):
    if not text:
        return text
    
    indent_str = ' ' * num
    lines = text.split('\n')
    
    if indent_first_line:
        indented_lines = [f"{indent_str}{line}" for line in lines]
    else:
        indented_lines = [lines[0]] + [f"{indent_str}{line}" for line in lines[1:]]
    
    return '\n'.join(indented_lines).strip()

env.filters['comment_hash'] = comment_hash
env.filters['comment_slash'] = comment_slash
env.filters['comment_xml'] = comment_xml
env.filters['comment_text'] = comment_text
env.filters['indent'] = indent

PRESERVED_JINJA2_CODEGEN_START_SLASH = "// PRESERVED_JINJA2_CODEGEN_START"
PRESERVED_JINJA2_CODEGEN_END_SLASH = "// PRESERVED_JINJA2_CODEGEN_END"
PRESERVED_JINJA2_CODEGEN_START_HASH = "# PRESERVED_JINJA2_CODEGEN_START"
PRESERVED_JINJA2_CODEGEN_END_HASH = "# PRESERVED_JINJA2_CODEGEN_END"
PRESERVED_JINJA2_CODEGEN_START_XML = "<!-- PRESERVED_JINJA2_CODEGEN_START -->"
PRESERVED_JINJA2_CODEGEN_END_XML = "<!-- PRESERVED_JINJA2_CODEGEN_END -->"

def fill_interface(interface: Interface, io: str):
    if interface.msg:
        interface.interface_type = "msg"
    elif interface.srv:
        interface.interface_type = "srv"
    elif interface.action:
        interface.interface_type = "action"
    io_type = io
    if not interface.name:
        interface.name = interface.msg or interface.srv or interface.action
    if not interface.namespace:
        interface.namespace = '.'.join(interface.name.split('/')[:-1])
    if not interface.dependency:
        interface.dependency = interface.type.split('/')[0]

def find_dependencies_of_fields(fields: List[Field]):
    dependencies = set()
    for field in fields:
        field_header = field.type.split('/')
        if len(field_header) == 1:
            field_header = None
        else:
            field_header = field_header[0]
        field.dependency = field.dependency or field_header
        if field.dependency:
            dependencies.add(field.dependency)
    return dependencies

def process_msg(msg: Msg, ppath=".", indent=0):
    dependencies = set(msg.dependencies or [])
    dependencies.update(find_dependencies_of_fields(msg.fields))
    msg.dependencies = list(dependencies)
    msg_path = f"{ppath}/msg/{msg.msg}.msg"
    preserving_jinja2_codegen("msg", "../templates/interface/msg.template.msg", msg_path, {
        "msg": msg
    }, overwrite=True)

def process_srv(srv: Srv, ppath=".", indent=0):
    dependencies = set(srv.dependencies or [])
    dependencies.update(find_dependencies_of_fields(srv.requests))
    dependencies.update(find_dependencies_of_fields(srv.responses))
    srv.dependencies = list(dependencies)
    srv_path = f"{ppath}/srv/{srv.srv}.srv"
    preserving_jinja2_codegen("srv", "../templates/interface/srv.template.srv", srv_path, {
        "srv": srv
    }, overwrite=True)

def process_action(action: Action):
    dependencies = set(action.dependencies or [])
    dependencies.update(find_dependencies_of_fields(action.goal_requests))
    dependencies.update(find_dependencies_of_fields(action.responses))
    dependencies.update(find_dependencies_of_fields(action.feedbacks))
    action.dependencies = list(dependencies)
    action_path = f"{ppath}/action/{action.action}.action"
    preserving_jinja2_codegen("action", "../templates/interface/action.template.action", action_path, {
        "action": action
    }, overwrite=True)

def preprocess_interface_universe(content: str):
    substitutions = {}
    pattern = r'^\$(\w+):(.*?)(?=^\$|\Z|^---)'
    for key, value in re.findall(pattern, content, re.MULTILINE | re.DOTALL):
        substitutions[key] = value
    if not substitutions:
        return content
    content = content.split("---", 1)[1]
    for key, value in substitutions.items():
        lines = value.strip().split("\n")
        for i in range(1, len(lines)):
            lines[i] = lines[i][2:]
        substitutions[key] = lines
    for key, value in substitutions.items():
        while True:
            key_start = content.find(f"${key}")
            if key_start == -1:
                break
            leftmost = content[:key_start].rfind("\n")
            indent = key_start - leftmost - 1
            content = content.replace(f"${key}", f"\n{' ' * indent}".join(value), 1)
    return content

def propcess_meta_config(meta_config: MetaConfig, path=".", indent=0):
    indentation = ' ' * indent
    print(indentation + f"[Processing] meta_config: {meta_config.workspace_path}")

    workspace_path = meta_config.workspace_path
    meta_config.assets_path.replace("${workspace_path}", workspace_path)
    meta_config.data_path.replace("${workspace_path}", workspace_path)

def preserving_jinja2_codegen(file_format: str, source_path: str, target_path: str, kwargs: Dict, overwrite=False):
    if not os.path.isfile(target_path) or overwrite:
        cp @(source_path) @(target_path)
        with open(target_path, "r") as f:
            template = env.from_string(f.read())
        with open(target_path, "w") as f:
            f.write(template.render(**kwargs))
        return

    with open(target_path, "r") as f:
        target_content = f.read()

    if file_format in {"xml", "md"}:
        start_marker = PRESERVED_JINJA2_CODEGEN_START_XML
        end_marker = PRESERVED_JINJA2_CODEGEN_END_XML
    elif file_format in {"hash", "python", "yaml", "toml", "txt"}:
        start_marker = PRESERVED_JINJA2_CODEGEN_START_HASH
        end_marker = PRESERVED_JINJA2_CODEGEN_END_HASH
    elif file_format in {"slash", "cpp", "rust"}:
        start_marker = PRESERVED_JINJA2_CODEGEN_START_SLASH
        end_marker = PRESERVED_JINJA2_CODEGEN_END_SLASH

    preserved_buffer = []

    while True:
        start_idx = target_content.find(start_marker) + len(start_marker)
        end_idx = target_content.find(end_marker)

        if start_idx == -1 or end_idx == -1 or start_idx >= end_idx:
            break

        inner_content = target_content[start_idx:end_idx]
        preserved_buffer.append(inner_content)
        target_content = target_content[end_idx:]

    rm -rf @(target_path)
    cp @(source_path) @(target_path)

    with open(target_path, "r") as f:
        target_content = f.read()

    offset = 0

    for content in preserved_buffer:
        start_idx = target_content[offset:].find(start_marker, offset) + len(start_marker) + offset
        end_idx = target_content[offset:].find(end_marker, offset) + offset

        if start_idx == -1 or end_idx == -1 or start_idx >= end_idx:
            raise ValueError("Markers not found in template")

        content = f"{content}"
        target_content = target_content[:start_idx] + content + target_content[end_idx:]
        offset += len(content)

    with open(target_path, "w") as f:
        f.write(env.from_string(target_content).render(**kwargs))

def process_universe(universe: Universe, meta_config: MetaConfig, path=".", indent=0):
    for item in universe.universe:
        if isinstance(item, Module):
            process_module(item, meta_config, path=path, indent=indent)
        elif isinstance(item, Pkg):
            process_pkg(item, meta_config, path=path, indent=indent)
        elif isinstance(item, Repo):
            process_repo(item, meta_config, path=path, indent=indent)

def process_module(module: Module, meta_config: MetaConfig, path=".", indent=0):
    indentation = ' ' * indent
    print(indentation + f"[Processing] module: {module.module}")

    mpath = f"{path}/{module.module}"
    if not os.path.isdir(mpath):
        print(indentation + f"[+] Creating module directory: {path}/{module.module}")
        mkdir -p @(mpath)

    if not module.children:
        return
    for item in module.children:
        if isinstance(item, Module):
            process_module(item, meta_config, path=mpath, indent=indent+4)
        elif isinstance(item, Pkg):
            process_pkg(item, meta_config, path=mpath, indent=indent+4)
        elif isinstance(item, Repo):
            process_repo(item, meta_config, path=mpath, indent=indent+4)

def process_repo(repo: Repo, meta_config: MetaConfig, path=".", indent=0):
    indentation = ' ' * indent
    print(indentation + f"[Processing] repo: {repo.repo}")

    url = repo.url
    rpath = f"{path}/{repo.repo}"

    meta_file_path = f"{rpath}/.ssu2_universe_repo_meta"
    meta_file = os.path.join(rpath, ".ssu2_universe_repo_meta")

    if os.path.isdir(rpath) and os.path.exists(meta_file):
        try:
            with open(meta_file, 'r') as f:
                existing_meta = f.read().strip()
            
            current_meta = f"url:{repo.url}|version:{repo.version}|subpaths:{','.join(repo.subpaths) if repo.subpaths else ''}"
            
            if existing_meta == current_meta:
                print(indentation + f"[-] Skipping repository (no changes): {url}")
                return
            else:
                print(indentation + f"[!] Repository config changed, reprocessing: {url}")
                # 기존 디렉토리 삭제
                shutil.rmtree(rpath)
        except Exception as e:
            print(indentation + f"[WARNING] Error reading meta file, reprocessing: {e}")
            # 메타파일 읽기 실패 시 재처리
            if os.path.exists(rpath):
                shutil.rmtree(rpath)
    print(indentation + f"[Processing] repo: {url}")

    touch @(meta_file_path)

    if not repo.subpaths:
        if repo.version:
            print(indentation + f"[+] Cloning {url} at version {repo.version} into {rpath}")
            git clone --branch @(repo.version) @(url) @(rpath)
        else:
            print(indentation + f"[+] Cloning {url} into {rpath}")
            git clone @(url) @(rpath)
        rm -rf @(rpath)/.git
        rm -rf @(rpath)/.github

        meta_content = f"url:{repo.url}|version:{repo.version}|subpaths:"
        with open(meta_file_path, "w") as f:
            f.write(meta_content)
        return
    try:
        temp_dir = tempfile.mkdtemp(prefix="repo_clone_", dir="/tmp")
        if repo.version:
            print(indentation + f"[+] Cloning {url} at version {repo.version} into {rpath} with subpaths")
            clone_cmd = f"git clone --filter=blob:none --sparse --branch {repo.version} {url} {temp_dir}/full_repo"
            # --filter=blob:none
            # git clone --filter=blob:none --sparse --branch main https://github.com/autowarefoundation/autoware_universe.git
        else:
            print(indentation + f"[+] Cloning {url} into {rpath} with subpaths")
            clone_cmd = f"git clone --filter=blob:none --sparse {url} {temp_dir}/full_repo"

        result = os.system(clone_cmd)
        if result != 0:
            print(indentation + f"[ERROR] Failed to clone {url}")
            return

        os.makedirs(rpath, exist_ok=True)

        # --- NEW: sparse-checkout enable ---
        os.system(f"git -C {temp_dir}/full_repo sparse-checkout init --cone")

        subpaths_str = " ".join(repo.subpaths)
        os.system(f"git -C {temp_dir}/full_repo sparse-checkout set {subpaths_str}")
        # --- END NEW ---

        for subpath in repo.subpaths:
            source_path = os.path.join(f"{temp_dir}/full_repo", subpath)
            if os.path.exists(source_path):
                # dest_name = os.path.basename(subpath.rstrip('/'))
                # dest_path = os.path.join(rpath, dest_name)
                dest_path = os.path.join(rpath, subpath)
                os.makedirs(os.path.dirname(dest_path), exist_ok=True)
                
                if os.path.isdir(source_path):
                    shutil.copytree(source_path, dest_path, dirs_exist_ok=True)
                else:
                    shutil.copy2(source_path, dest_path)
                print(indentation + f"  [✓] Copied: {subpath}")
            else:
                print(indentation + f"  [⚠] Subpath not found: {subpath}")
        
        meta_content = f"url:{repo.url}|version:{repo.version}|subpaths:{','.join(repo.subpaths)}"
        with open(meta_file_path, "w") as f:
            f.write(meta_content)

    except Exception as e:
        print(indentation + f"[ERROR] Failed: {str(e)}")
    finally:
        # 임시 디렉토리 정리
        if os.path.exists(temp_dir):
            shutil.rmtree(temp_dir)
            print(f"{indentation}[+] Cleaned temp directory")

def process_interface_pkg(pkg: Pkg, meta_config: MetaConfig, path=".", ppath=".", indent=0):
    if pkg.msgs and not os.path.isdir(f"{ppath}/msg"):
        mkdir -p @(ppath)/msg
    if pkg.srvs and not os.path.isdir(f"{ppath}/srv"):
        mkdir -p @(ppath)/srv
    if pkg.actions and not os.path.isdir(f"{ppath}/action"):
        mkdir -p @(ppath)/action

    dependencies = set(pkg.dependencies or [])
    if pkg.msgs:
        for msg in pkg.msgs:
            process_msg(msg, ppath=ppath, indent=indent+4)
            dependencies.update(msg.dependencies)
    if pkg.srvs:
        for srv in pkg.srvs:
            process_srv(srv, ppath=ppath, indent=indent+4)
            dependencies.update(srv.dependencies)
    if pkg.actions:
        for action in pkg.actions:
            process_action(action, ppath=ppath, indent=indent+4)
            dependencies.update(action.dependencies)
    pkg.dependencies = list(dependencies)
    for dependency in pkg.dependencies:
        if dependency == pkg.pkg:
            pkg.dependencies.remove(dependency)
    
    cmake_lists_path = f"{ppath}/CMakeLists.txt"
    preserving_jinja2_codegen("hash", "../templates/interface/CMakeLists.template.txt", cmake_lists_path, {
        "pkg": pkg
    })

def process_node_pkg(pkg: Pkg, meta_config: MetaConfig, path=".", ppath=".", indent=0):
    dependencies = set(pkg.dependencies or [])
    for interface in pkg.inputs:
        fill_interface(interface, "input")
        dependencies.add(interface.dependency)
    for interface in pkg.outputs:
        fill_interface(interface, "output")
        dependencies.add(interface.dependency)
    pkg.dependencies = list(dependencies)

    if pkg.lang.name == "cpp":
        cmakelists_path = f"{ppath}/CMakeLists.txt"
        preserving_jinja2_codegen("hash", "../templates/CMakeLists.template.txt", cmakelists_path, {
            "pkg": pkg
        })
    elif pkg.lang.name == "rust":
        cargo_toml_path = f"{ppath}/Cargo.toml"
        preserving_jinja2_codegen("toml", "../templates/Cargo.template.toml", cargo_toml_path, {
            "pkg": pkg
        })
    readme_path = f"{ppath}/README.md"
    preserving_jinja2_codegen("md", "../templates/README.pkg.template.md", readme_path, {
        "pkg": pkg
    }, overwrite=True)

def process_algorithm_pkg(pkg: Pkg, meta_config: MetaConfig, path=".", ppath=".", indent=0):
    if pkg.lang.name == "cpp":
        cmakelists_path = f"{ppath}/CMakeLists.txt"
        preserving_jinja2_codegen("hash", "../templates/CMakeLists.template.txt", cmakelists_path, {
            "pkg": pkg
        })
    elif pkg.lang.name == "rust":
        cargo_toml_path = f"{ppath}/Cargo.toml"
        preserving_jinja2_codegen("toml", "../templates/Cargo.template.toml", cargo_toml_path, {
            "pkg": pkg
        })
        if not os.path.isdir(f"{ppath}/src"):
            mkdir -p @(ppath)/src
        if pkg.source_codes:
            for source_code in pkg.source_codes:
                source_code_path = f"{ppath}/src/{source_code}.rs"
                touch @(source_code_path)
    readme_path = f"{ppath}/README.md"
    preserving_jinja2_codegen("md", "../templates/README.pkg.template.md", readme_path, {
        "pkg": pkg
    }, overwrite=True)

def process_launch_pkg(pkg: Pkg, meta_config: MetaConfig, path=".", ppath=".", indent=0):
    if pkg.install_to_share is not None:
        for share_path in pkg.install_to_share:
            spath = f"{ppath}/{share_path}"
            if not os.path.isdir(spath):
                mkdir -p @(spath)
    
    lpath = f"{ppath}/launch"
    if not os.path.isdir(lpath):
        mkdir -p @(lpath)
    launch_path = f"{lpath}/{pkg.pkg}.launch.xml"
    preserving_jinja2_codegen("xml", "../templates/launch/template.launch.xml", launch_path, {
        "pkg": pkg
    })

    pkg.install_to_share.append("launch")
    cmakelists_path = f"{ppath}/CMakeLists.txt"
    preserving_jinja2_codegen("hash", "../templates/launch/CMakeLists.template.txt", cmakelists_path, {
        "pkg": pkg
    })

def process_pkg(pkg: Pkg, meta_config: MetaConfig, path=".", indent=0):
    indentation = ' ' * indent
    print(indentation + f"[Processing] pkg: {pkg.pkg}")

    ppath = f"{path}/{pkg.pkg}"
    if not os.path.isdir(ppath):
        print(indentation + f"[+] Creating pkg directory: {path}/{pkg.pkg}")
        mkdir -p @(ppath)

    if not pkg.description:
        pkg.description = pkg.purpose
    if not pkg.cmake_version:
        pkg.cmake_version = meta_config.cmake_version
    if not pkg.rust_edition:
        pkg.rust_edition = meta_config.rust_edition
    if not pkg.version:
        pkg.version = meta_config.pkg_default_version
    for maintainer in meta_config.maintainers:
        pkg.maintainers.append(maintainer)
    for author in meta_config.authors:
        pkg.authors.append(author)
    if not pkg.license:
        pkg.license = meta_config.license

    packge_xml_path = f"{ppath}/package.xml"
    if pkg.type.value == "launch":
        process_launch_pkg(pkg, meta_config, path=path, ppath=ppath, indent=indent)
        preserving_jinja2_codegen("xml", "../templates/launch/package.template.xml", packge_xml_path, {
            "pkg": pkg
        })
    elif pkg.type.value == "algorithm":
        process_algorithm_pkg(pkg, meta_config, path=path, ppath=ppath, indent=indent)
        preserving_jinja2_codegen("xml", "../templates/algorithm/package.template.xml", packge_xml_path, {
            "pkg": pkg
        })
    elif pkg.type.value == "interface":
        process_interface_pkg(pkg, meta_config, path=path, ppath=ppath, indent=indent)
        preserving_jinja2_codegen("xml", "../templates/interface/package.template.xml", packge_xml_path, {
            "pkg": pkg
        })
    elif pkg.type.value == "node":
        process_node_pkg(pkg, meta_config, path=path, ppath=ppath, indent=indent)
        preserving_jinja2_codegen("xml", "../templates/node/package.template.xml", packge_xml_path, {
            "pkg": pkg
        })

def generate_vehicle_universe(meta_config: MetaConfig, path="."):
    vehicle_path = f"{path}/vehicle"

    with open("../vehicle_presets.yaml", "r") as f:
        vehicle_presets = yaml.safe_load(f)

    for (vehicle_type, vehicle_params) in vehicle_presets["vehicle_presets"].items():
        # YAML에 색상 정보가 없을 경우를 대비해 기본값 설정
        color_defaults = {
            'body_color_r': 0.8,
            'body_color_g': 0.2, 
            'body_color_b': 0.2,
            'window_color_r': 0.1,
            'window_color_g': 0.3,
            'window_color_b': 0.8
        }

        vehicle_params["name"] = vehicle_type
        vehicle_params["width"] = vehicle_params["wheel_tread"] + vehicle_params["left_overhang"] + vehicle_params["right_overhang"]
        vehicle_params["length"] = vehicle_params["wheel_base"] + vehicle_params["front_overhang"] + vehicle_params["rear_overhang"]
        vehicle_params["height"] = max([
            vehicle_params["front_bumper_height"],
            vehicle_params["a_pillar_height"],
            vehicle_params["front_roof_height"],
            vehicle_params["rear_roof_height"],
            vehicle_params["d_pillar_height"],
            vehicle_params["rear_bumper_height"]
        ])

        # YAML 파라미터에 색상 기본값 추가
        for key, value in color_defaults.items():
            if key not in vehicle_params:
                vehicle_params[key] = value
        
        params = VehicleParameters(**vehicle_params)
        print(f"Vehicle type: {vehicle_type}")
        print(f"  Body color: {params.body_color}")
        print(f"  Window color: {params.window_color}")

        container_path = f"{vehicle_path}/{vehicle_type}_vehicle_launch"
        if not os.path.exists(container_path):
            os.makedirs(container_path)

        description_path = f"{container_path}/{vehicle_type}_vehicle_description"
        if not os.path.exists(description_path):
            os.makedirs(description_path)

        package_path = f"{description_path}/package.xml"
        preserving_jinja2_codegen("xml", "../templates/vehicle/vehicle_description/package.template.xml", package_path, {
            "vehicle": vehicle_params,
            "version": meta_config.pkg_default_version,
            "maintainers": meta_config.maintainers,
            "authors": meta_config.authors,
            "license": meta_config.license
        })

        cmake_lists_path = f"{description_path}/CMakeLists.txt"
        preserving_jinja2_codegen("hash", "../templates/vehicle/vehicle_description/CMakeLists.template.txt", cmake_lists_path, {
            "vehicle": vehicle_params,
            "cmake_version": meta_config.cmake_version
        }, overwrite=True)

        config_path = f"{description_path}/config"
        if not os.path.exists(config_path):
            os.makedirs(config_path)

        vehicle_info_param_yaml_path = f"{config_path}/vehicle_info.param.yaml"
        with open(vehicle_info_param_yaml_path, "w") as f:
            f.write("ros__parameters:\n")
            f.write(f"  vehicle_name: {vehicle_params['name']}\n")
            f.write(f"  wheel_base: {vehicle_params['wheel_base']}\n")
            f.write(f"  wheel_tread: {vehicle_params['wheel_tread']}\n")
            f.write(f"  wheel_radius: {vehicle_params['wheel_radius']}\n")
            f.write(f"  wheel_width: {vehicle_params['wheel_width']}\n")
            f.write(f"  front_overhang: {vehicle_params['front_overhang']}\n")
            f.write(f"  rear_overhang: {vehicle_params['rear_overhang']}\n")
            f.write(f"  left_overhang: {vehicle_params['left_overhang']}\n")
            f.write(f"  right_overhang: {vehicle_params['right_overhang']}\n")
            f.write(f"  ground_offset: {vehicle_params['ground_offset']}\n")
            f.write(f"  vehicle_height: {vehicle_params['height']}\n")
            f.write(f"  vehicle_width: {vehicle_params['width']}\n")
            f.write(f"  vehicle_length: {vehicle_params['length']}\n")
            # f.write(f"  max_steer_angle: {vehicle_params['max_steer_angle']}\n")
        
        mesh_path = f"{description_path}/mesh"
        if not os.path.exists(mesh_path):
            os.makedirs(mesh_path)

        # 간단한 버전 생성 (VehicleParameters의 색상 사용)
        export_to_obj_simple(params, f"{mesh_path}/vehicle_{vehicle_type}_simple.obj")
        print(f"  Exported to vehicle_{vehicle_type}_simple.obj and vehicle_{vehicle_type}_simple.mtl")
        print()
        
        urdf_path = f"{description_path}/urdf"
        if not os.path.exists(urdf_path):
            os.makedirs(urdf_path)

        vehicle_urdf_path = f"{urdf_path}/vehicle.xacro"
        preserving_jinja2_codegen("xml", "../templates/vehicle/vehicle_description/urdf/vehicle.template.xacro", vehicle_urdf_path, {
            "vehicle": vehicle_params
        }, overwrite=True)

        vehicle_launch_path = f"{container_path}/{vehicle_type}_vehicle_launch"
        if not os.path.exists(vehicle_launch_path):
            os.makedirs(vehicle_launch_path)

        package_path = f"{vehicle_launch_path}/package.xml"
        preserving_jinja2_codegen("xml", "../templates/vehicle/vehicle_launch/package.template.xml", package_path, {
            "vehicle": vehicle_params,
            "version": meta_config.pkg_default_version,
            "maintainers": meta_config.maintainers,
            "authors": meta_config.authors,
            "license": meta_config.license
        })

        cmake_lists_path = f"{vehicle_launch_path}/CMakeLists.txt"
        preserving_jinja2_codegen("hash", "../templates/vehicle/vehicle_launch/CMakeLists.template.txt", cmake_lists_path, {
            "vehicle": vehicle_params,
            "cmake_version": meta_config.cmake_version
        }, overwrite=True)

        launch_path = f"{vehicle_launch_path}/launch"
        if not os.path.exists(launch_path):
            os.makedirs(launch_path)

        vehicle_interface_path = f"{launch_path}/vehicle_interface.launch.xml"
        preserving_jinja2_codegen("xml", "../templates/vehicle/vehicle_launch/launch/vehicle_interface.launch.template.xml", vehicle_interface_path, {
            "vehicle": vehicle_params
        }, overwrite=True)

def main():
    YAML_PATH = ".."
    with open(f"{YAML_PATH}/universe.yaml", "r") as f:
        universe = Universe(**yaml.safe_load(f))
    
    with open(f"{YAML_PATH}/interface_universe.yaml", "r", encoding="utf-8") as f:
        content = preprocess_interface_universe(f.read())
        content = yaml.safe_load(content)
        interface_universe = Universe(**content)

    with open(f"{YAML_PATH}/repos_universe.yaml", "r") as f:
        repos_universe = Universe(**yaml.safe_load(f))

    with open(f"{YAML_PATH}/meta_config.yaml", "r") as f:
        meta_config = MetaConfig(**yaml.safe_load(f))

    process_universe(repos_universe, meta_config, "../src")
    process_universe(interface_universe, meta_config, "../ssu2_msgs")
    process_universe(universe, meta_config, "../src")

    generate_vehicle_universe(meta_config, "../src")

if __name__ == "__main__":
    main()