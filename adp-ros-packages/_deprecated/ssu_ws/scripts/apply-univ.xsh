#!/usr/bin/env xonsh

from pydantic import BaseModel, Field
from typing import List, Optional, Union
from pathlib import Path
import yaml
import re

UNIVERSE_YAML = '../universe.yaml'
_README_TEMPLATE_MD = '_README.template.md'
_README_TEMPLATE_MD_PATH = f'../template/{_README_TEMPLATE_MD}'

md_heading = re.compile(r'^#+\s+.+$')
md_list_indenting = re.compile(r'^\s*-\s+.+$')
def add_md_headings(s, heading=2):
    res = ""
    for line in s.split("\n"):
        res += add_md_heading_by_line(line, heading=heading).strip() + "\n"
    return res
def add_md_heading_by_line(s, heading=2):
    if md_heading.match(s):
        return "#"*heading + s
    return s
def add_md_list_indentings(s, list_indenting=2):
    res = ""
    for line in s.split("\n"):
        res += add_md_list_indenting_by_line(line, list_indenting=list_indenting)
    return res
def add_md_list_indenting_by_line(s, list_indenting=2):
    if md_list_indenting.match(s):
        return " "*list_indenting + s
    return s

mustache_pattern = re.compile(r'\{\{\s*(\w+)\s*\}\}')

def preprocess_mustache_template(template):
    """
    1. 일반 {, } 이스케이프 처리
    2. {{, }}를 {, }로 변환
    """
    
    return mustache_pattern.sub(r'{\1}', template)

def render_mustache_template(template, **context):
    """
    전처리 후 f-string으로 렌더링
    """
    # 템플릿 전처리
    processed_template = preprocess_mustache_template(template)
    
    # f-string으로 렌더링
    try:
        return processed_template.format(**context)
    except KeyError as e:
        # 없는 변수는 원래 형태로 유지
        return processed_template

Item = Union['Module', 'Pkg', 'Repo']

class Universe(BaseModel):
    universe: List[Item]

class Module(BaseModel):
    module: str
    children: Optional[List[Item]] = None

    def process(self, path="", indent=0):
        print(" "*indent + f"Processing module: {self.module}")
        mpath = f"{path}/{self.module}" if path else self.module
        mkdir -p @(mpath)
        if self.children:
            for child in self.children:
                if isinstance(child, Module):
                    child.process(path=mpath, indent=indent+4)
                elif isinstance(child, Pkg):
                    child.process(path=mpath, indent=indent+4)
                elif isinstance(child, Repo):
                    child.process(path=mpath, indent=indent+4)

Msgs = Union['Msg', 'Srv', 'Action']
ParamItem = Union['Param', 'ParamGroup']

class Pkg(BaseModel):
    pkg: str
    lang: Optional[str] = None
    type_: Optional[str] = Field(alias='type')
    purpose: Optional[str] = None
    design: Optional[str] = None
    inputs: Optional[List[Msgs]] = None
    outputs: Optional[List[Msgs]] = None
    parameters: Optional[List[ParamItem]] = None
    references: Optional[List[str]] = None

    def process(self, path="", indent=0):
        print(" "*indent + f"Processing pkg: {self.pkg}")
        ppath = f"{path}/{self.pkg}" if path else self.pkg
        deps = []

        # modify heading level of purpose, design section
        if self.purpose:
            self.purpose = add_md_headings(self.purpose, heading=2).strip()
        if self.design:
            self.design = add_md_headings(self.design, heading=2).strip()

        # make md string for inputs, outputs, parameters, references
        s = ""
        if self.inputs:
            for msgs in self.inputs:
                if isinstance(msgs, Msg):
                    s += msgs.process(path=ppath, indent=indent+4, heading=3, list_indenting=2)
                elif isinstance(msgs, Srv):
                    s += msgs.process(path=ppath, indent=indent+4, heading=3, list_indenting=2)
                elif isinstance(msgs, Action):
                    s += msgs.process(path=ppath, indent=indent+4, heading=3, list_indenting=2)
        self.inputs = s
        s = ""
        if self.outputs:
            for msgs in self.outputs:
                if isinstance(msgs, Msg):
                    s += msgs.process(path=ppath, indent=indent+4, heading=3, list_indenting=2)
                elif isinstance(msgs, Srv):
                    s += msgs.process(path=ppath, indent=indent+4, heading=3, list_indenting=2)
                elif isinstance(msgs, Action):
                    s += msgs.process(path=ppath, indent=indent+4, heading=3, list_indenting=2)
        self.outputs = s
        s = ""
        if self.parameters:
            is_first_param = True
            for param_item in self.parameters:
                if isinstance(param_item, Param):
                    s += param_item.process(path=ppath, indent=indent+4, is_first_param=is_first_param)
                    is_first_param = False
                elif isinstance(param_item, ParamGroup):
                    s += param_item.process(path=ppath, indent=indent+4, heading=2)
        self.parameters = s
        s = ""
        if self.references:
            for ref in self.references:
                s += f"- {ref}\n"
        self.references = s

        # make pkg and configure packge.xml and addition according to lang
        package_xml = f"{ppath}/package.xml"
        if self.lang == 'rust':
            deps.append('rclrs')
            ros2 pkg create --build-type ament_cmake @(ppath) --dependencies @(" ".join(deps))
            cargo_toml = f"{ppath}/Cargo.toml"

            sed -i 's/ament_cmake/ament_cargo/g' @(package_xml)
            sed -i "/<test_depend>ament_lint_auto</test_depend>/d" @(package_xml)
            sed -i "/<test_depend>ament_lint_common</test_depend>/d" @(package_xml)
            
            if Path(cargo_toml).exists():
                print(" "*indent + "! Skipping: Cargo.toml already exists")
            else:
                cargo init
                cargo add "rclrs@0.6"
        elif self.lang == 'cpp':
            deps.append('rclcpp')
            ros2 pkg create --build-type ament_cmake @(ppath) --dependencies @(" ".join(deps))
        
        if Path(ppath).exists():
            # load md template and render
            _readme = f"{ppath}/{_README_TEMPLATE_MD}"
            if not Path(_readme).exists():
                cp @(_README_TEMPLATE_MD_PATH) @(ppath)
            with open(_readme, 'r', encoding='utf-8') as f:
                readme_template = f.read()
            readme_template = preprocess_mustache_template(readme_template)
            readme = f"{ppath}/README.md"
            if not Path(readme).exists():
                touch @(readme)
            with open(readme, 'w', encoding='utf-8') as f:
                rendered_readme = render_mustache_template(readme_template, pkg=self.pkg, purpose=self.purpose, design=self.design, inputs=self.inputs, outputs=self.outputs, parameters=self.parameters, references=self.references)
                f.write(rendered_readme)

class Repo(BaseModel):
    repo: str
    dist: str
    version: str
    subpaths: Optional[List[str]] = None

    def process(self, path="", indent=0):
        print(" "*indent + f"Processing repo: {self.repo}")
        url = self.repo
        dist = f"{path}/{self.dist}"
        version = self.version
        if Path(dist).exists() and any(Path(dist).iterdir()):
            print(" "*indent + f"! Skipping: {dist} already exists and is not empty")
            return
        if not self.subpaths:
            print(" "*indent + f"Cloning {url} at version {version} into {dist}")
            git clone --branch @(version) @(url) @(dist)
            rm -rf @(dist)/.git
            rm -rf @(dist)/.github
        else:
            print(" "*indent + f"Processing subpaths for {url}")
            mkdir -p @(dist)
            cd @(dist)
            git init
            git remote add origin @(url)
            git config core.sparseCheckout true
            sparse_checkout_file = f"{dist}/.git/info/sparse-checkout"
            with open(sparse_checkout_file, 'w', encoding='utf-8') as f:
                for subpath in self.subpaths:
                    f.write(f"{subpath}\n")
            git pull origin @(version)
            rm -rf .git
            cd -

class Msg(BaseModel):
    msg: str
    type_: str = Field(alias='type')
    description: Optional[str] = None

    def process(self, path="", indent=0, heading=2, list_indenting=2):
        print(" "*indent + f"Processing msg: {self.msg}")
        s = f"- **{self.msg}**: `{self.type_}`: "
        if self.description:
            md_heading_added = add_md_headings(self.description, heading=heading)
            s += add_md_list_indentings(md_heading_added, list_indenting=list_indenting).strip()
        s += "\n"
        return s

class Srv(BaseModel):
    srv: str
    type_: str = Field(alias='type')
    description: Optional[str] = None

    def process(self, path="", indent=0, heading=2, list_indenting=2):
        print(" "*indent + f"Processing srv: {self.srv}")
        s = f"- **{self.srv}**: `{self.type_}`: "
        if self.description:
            md_heading_added = add_md_headings(self.description, heading=heading)
            s += add_md_list_indentings(md_heading_added, list_indenting=list_indenting).strip()
        s += "\n"
        return s

class Action(BaseModel):
    action: str
    type_: str = Field(alias='type')
    description: Optional[str] = None

    def process(self, path="", indent=0, heading=2, list_indenting=2):
        print(" "*indent + f"Processing action: {self.action}")
        s = f"- **{self.action}**: `{self.type_}`: "
        if self.description:
            md_heading_added = add_md_headings(self.description, heading=heading)
            s += add_md_list_indentings(md_heading_added, list_indenting=list_indenting).strip()
        s += "\n"
        return s

class Param(BaseModel):
    name: str
    type_: str = Field(alias='type')
    unit: Optional[str] = None
    description: Optional[str] = None
    default: Optional[Union[str, int, float]] = None

    def process(self, path="", indent=0, is_first_param=False):
        print(" "*indent + f"Processing param: {self.name}")
        s = ""
        if is_first_param:
            s += "Name | Type | Description | Unit | Default\n"
            s += "--- | --- | --- | --- | ---\n"
        if self.description:
            self.description = self.description.strip()
        s += f"{self.name} | {self.type_} | {self.description} | {self.unit} | {self.default}\n"
        return s

class ParamGroup(BaseModel):
    group: str
    children: Optional[List[ParamItem]] = None

    def process(self, path="", indent=0, heading=2):
        print(" "*indent + f"Processing param group: {self.group}")
        s = ""
        s += "\n"
        s += add_md_heading_by_line(f"# {self.group}", heading=heading) + "\n"
        is_first_param = True
        for param_item in self.children:
            if isinstance(param_item, Param):
                s += param_item.process(path=path, indent=indent+4, is_first_param=is_first_param)
                is_first_param = False
            elif isinstance(param_item, ParamGroup):
                s += param_item.process(path=path, indent=indent+4, heading=heading+1)
        return s

with open(UNIVERSE_YAML, 'r', encoding='utf-8') as f:
    data = yaml.safe_load(f)

univ = Universe(**data)
SRC_DIR = "../src"
for item in univ.universe:
    if isinstance(item, Module):
        item.process(path=f"{SRC_DIR}")
    elif isinstance(item, Pkg):
        item.process(path=f"{SRC_DIR}")
    elif isinstance(item, Repo):
        item.process(path=f"{SRC_DIR}")