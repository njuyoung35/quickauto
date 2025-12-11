#!/usr/bin/env xonsh

import re
from pathlib import Path
from pydantic import BaseModel, Field
import pydantic
from typing import List, Optional, Union
import yaml

comment = re.compile(r'^#\s*(.*)')
horizontal_rule = re.compile(r'^[-\*]{3,}\s')
field = re.compile(r'^([^#\s]+)\s+([^#\s]+)(\s+#\s*(.*))?')

class MsgField(BaseModel):
    type_: str = Field(alias='type')
    name: str
    default_value: Optional[Union[str | int | float]] = None
    comment: str = ""

    def stringfy(self):
        s = ""
        s += f"          - name: {self.name}\n"
        s += f"            type: {self.type_}\n"
        if self.default_value:
            s += f"            default_value: {self.default_value}\n"
        s += f"            comment: |\n"
        for comment in self.comment.split("\n"):
            s += f"              {comment}\n"
        return s

def parse_file(file):
    comment_buffer = [[], [], []]
    remained_comment = []
    fields = [[], [], []]
    dependencies = set()
    mode = 0
    for line in file.split('\n'):
        comment_match = comment.match(line)
        if comment_match:
            content = comment_match.group(1)
            comment_buffer[mode].append(content)
            continue
        field_match = field.match(line)
        if field_match:
            type_ = field_match.group(1)
            name = field_match.group(2)
            latter_comment = field_match.group(4)
            if latter_comment:
                comment_buffer[mode].append(latter_comment)
            comment_str = "\n".join(comment_buffer[mode])
            default_value = None
            if name.find("=") != -1:
                name, default_value = name.split("=")
            fields[mode].append(MsgField(type=type_, name=name, default_value=default_value, comment=comment_str))
            if type_.find("/") != -1:
                dependencies.add(type_.split("/")[0])
            comment_buffer[mode] = []
            continue
        if comment_buffer[mode]:
            for line in comment_buffer[mode]:
                remained_comment.append(line)
            comment_buffer[mode] = []
        if horizontal_rule.match(line):
            mode += 1

    return remained_comment, fields, dependencies

class Msg(BaseModel):
    msg: str
    description: str = ""
    fields: List[MsgField] = []

class Srv(BaseModel):
    srv: str
    description: str = ""
    requests: List[MsgField] = []
    responses: List[MsgField] = []

class Action(BaseModel):
    action: str
    description: str = ""
    goal_requests: List[MsgField] = []
    responses: List[MsgField] = []
    feedback: List[MsgField] = []

class Package(BaseModel):
    pkg: str
    description: str = ""
    dependencies: List[str] = []
    msgs: List['Msg'] = []
    srvs: List['Srv'] = []
    actions: List['Action'] = []
    
Package.model_rebuild()

class MsgsUniverse(BaseModel):
    universe: List['Package'] = []

def read_package(package):
    msgs = []
    srvs = []
    actions = []
    package_description = ""
    dependencies = set()
    readme = ""
    try:
        with open(package / 'README.md') as f:
            readme = f.read()
    except FileNotFoundError:
        pass
    for subdir in Path(package).iterdir():
        if not subdir.is_dir():
            continue
        if subdir.name == 'msg':
            for file in subdir.iterdir():
                if file.suffix == '.msg':
                    name = file.stem
                    print(f"Parsing {file}")
                    with file.open() as f:
                        remained_comment, fields, dependencies_ = parse_file(f.read())
                        if not remained_comment:
                            pat = re.compile(f"^\\*.*{name}[^:]*:\\s*(.*)\\n$")
                            pat_match = pat.match(readme)
                            if pat_match:
                                remained_comment = [pat_match.group(1)]
                        if not remained_comment:
                            print("! Warning: no description for " + name)
                            remained_comment = ""
                        remained_comment = "\n".join(remained_comment)
                        msg = Msg(msg=name, description=remained_comment, fields=fields[0])
                        msgs.append(msg)
                        dependencies.update(dependencies_)
        elif subdir.name == 'srv':
            for file in subdir.iterdir():
                if file.suffix == '.srv':
                    name = file.stem
                    with file.open() as f:
                        remained_comment, fields, dependencies_ = parse_file(f.read())
                        if not remained_comment:
                            pat = re.compile(f"^\\*.*{name}[^:]*:\\s*(.*)\\n$")
                            pat_match = pat.match(readme)
                            if pat_match:
                                remained_comment = [pat_match.group(1)]
                        if not remained_comment:
                            print("! Warning: no description for " + name)
                            remained_comment = ""
                        remained_comment = "\n".join(remained_comment)
                        srv = Srv(srv=name, description=remained_comment, requests=fields[0], responses=fields[1])
                        srvs.append(srv)
                        dependencies.update(dependencies_)
        elif subdir.name == 'action':
            for file in subdir.iterdir():
                if file.suffix == '.action':
                    name = file.stem
                    with file.open() as f:
                        remained_comment, fields, dependencies_ = parse_file(f.read())
                        if not remained_comment:
                            pat = re.compile(f"^\\*.*{name}[^:]*:\\s*(.*)\\n$")
                            pat_match = pat.match(readme)
                            if pat_match:
                                remained_comment = [pat_match.group(1)]
                        if not remained_comment:
                            print("! Warning: no description for " + name)
                            remained_comment = ""
                        remained_comment = "\n".join(remained_comment)
                        action = Action(action=name, description=remained_comment, goal_requests=fields[0], responses=fields[1], feedback=fields[2])
                        actions.append(action)
                        dependencies.update(dependencies_)
    package_description = ""
    pkg = Package(pkg=package.name, description=package_description, dependencies=list(dependencies), msgs=msgs, srvs=srvs, actions=actions)
    return pkg

def read_all_packages(path):
    res = []
    for dir_ in Path(path).iterdir():
        if dir_.is_dir():
            res.append(read_package(dir_))
    return res

def process_msgs_universe(path, output_dist):

    pkgs = read_all_packages(path)

    s = "universe:\n"
    for pkg in pkgs:
        print(f"Processing {pkg.pkg}:")
        s += f"  - pkg: {pkg.pkg}\n"
        s += f"    description: {pkg.description} |\n"
        for line in pkg.description.split("\n"):
            s += f"      {line}\n"
        s += f"    dependencies: |\n"
        for dep in pkg.dependencies:
            s += f"      {dep}\n"
        s += f"    msgs:\n"
        for msg in pkg.msgs:
            s += f"      - msg: {msg.msg}\n"
            s += f"        description: |\n"
            for line in msg.description.split("\n"):
                s += f"          {line}\n"
            s += f"        fields:\n"
            for field in msg.fields:
                s += field.stringfy()
        s += f"    srvs:\n"
        for srv in pkg.srvs:
            s += f"      - srv: {srv.srv}\n"
            s += f"        description: |\n"
            for line in msg.description.split("\n"):
                s += f"          {line}\n"
            s += f"        requests:\n"
            for field in srv.requests:
                s += field.stringfy()
            s += f"        responses:\n"
            for field in srv.responses:
                s += field.stringfy()
        s += f"    actions:\n"
        for action in pkg.actions:
            s += f"      - action: {action.action}\n"
            s += f"        description: |\n"
            for line in msg.description.split("\n"):
                s += f"          {line}\n"
            s += f"        goal_requests:\n"
            for field in action.goal_requests:
                s += field.stringfy()
            s += f"        responses:\n"
            for field in action.responses:
                s += field.stringfy()
            s += f"        feedback:\n"
            for field in action.feedback:
                s += field.stringfy()

    touch @(output_dist)
    with open(output_dist, "w") as f:
        f.write(s)

def description_summarizer(description):
    s = ""
    if description:
        desc = description.split("\n")[0]
        if len(desc) > 50:
            desc = desc[:50] + "..."
        s += f"{desc}"
    return s

def comment_summarizer(comment):
    s = ""
    if comment:
        comm = comment.split("\n")[0]
        if len(comm) > 60:
            comm = comm[:60] + "..."
        s += comm
    return s

def field_summarizer(field):
    s = ""
    comment = comment_summarizer(field.comment)
    default_value = field.default_value if field.default_value else ""
    s += f"| {field.name} | `{field.type_}` | {comment} | {default_value} |\n"
    return s

def summary(path, output_dist):
    pkgs = read_all_packages(path)

    s = ""
    for pkg in pkgs:
        print(f"Processing {pkg.pkg}:")
        s += f"#### **{pkg.pkg}**\n"
        if pkg.description:
            s += f"{pkg.description}\n\n"
        if pkg.dependencies:
            s += f"##### Dependencies\n"
            for dep in pkg.dependencies:
                s += f"- {dep}\n"

        if pkg.msgs:
            s += f"##### Messages\n"
            for msg in pkg.msgs:
                s += f"###### **{msg.msg}**"
                dsc = description_summarizer(msg.description)
                if dsc:
                    s += "\n"
                    s += dsc
                s += f"\n\n"
                if msg.fields:
                    s += f"- Fields\n\n"
                    s += f"| Name | Type | Description | Default |\n"
                    s += f"| --- | --- | --- | --- |\n"
                    for field in msg.fields:
                        s += field_summarizer(field)
                    s += "\n"
        if pkg.srvs:
            s += f"##### Services\n"
            for srv in pkg.srvs:
                s += f"###### **{srv.srv}**"
                dsc = description_summarizer(srv.description)
                if dsc:
                    s += "\n"
                    s += dsc
                s += f"\n\n"
                if srv.requests:
                    s += f"- Requests\n\n"
                    s += f"| Name | Type | Description | Default |\n"
                    s += f"| --- | --- | --- | --- |\n"
                    for field in srv.requests:
                        s += field_summarizer(field)
                    s += "\n"
                if srv.responses:
                    s += f"- Responses\n\n"
                    s += f"| Name | Type | Description | Default |\n"
                    s += f"| --- | --- | --- | --- |\n"
                    for field in srv.responses:
                        s += field_summarizer(field)
                    s += "\n"
        if pkg.actions:
            s += f"##### Actions\n"
            for action in pkg.actions:
                s += f"###### **{action.action}**"
                dsc = description_summarizer(action.description)
                if dsc:
                    s += "\n"
                    s += dsc
                s += f"\n\n"
                if action.goal_requests:
                    s += f"- Goal Requests\n\n"
                    s += f"| Name | Type | Description | Default |\n"
                    s += f"| --- | --- | --- | --- |\n"
                    for field in action.goal_requests:
                        s += field_summarizer(field)
                    s += "\n"
                if action.responses:
                    s += f"- Responses\n\n"
                    s += f"| Name | Type | Description | Default |\n"
                    s += f"| --- | --- | --- | --- |\n"
                    for field in action.responses:
                        s += field_summarizer(field)
                    s += "\n"
                if action.feedback:
                    s += f"- Feedback\n\n"
                    s += f"| Name | Type | Description | Default |\n"
                    s += f"| --- | --- | --- | --- |\n"
                    for field in action.feedback:
                        s += field_summarizer(field)
                    s += "\n"
    touch @(output_dist)
    with open(output_dist, "w") as f:
        f.write(s)

def txt_summary(path, output_dist):
    pkgs = read_all_packages(path)

    ss = ""
    for pkg in pkgs:
        print(f"Processing {pkg.pkg}:")
        ss += f"{pkg.pkg}/\n"
        if pkg.dependencies:
            ss += f"  dependencies:\n"
            for dep in pkg.dependencies:
                ss += f"    - {dep}\n"
        ss += "\n"

        if pkg.msgs:
            for msg in pkg.msgs:
                s = f"  - msg: {msg.msg}"
                dsc = description_summarizer(msg.description)
                if dsc:
                    s += f" # {dsc}"
                s += "\n"
                ss += s

                max_len = 0
                lines = []
                comments = {}
                if msg.fields:
                    for field in msg.fields:
                        s = f"    - {field.name}: {field.type_}"
                        if field.default_value:
                            s += f" (={field.default_value})"
                        if len(s) > max_len:
                            max_len = len(s)
                        if field.comment:
                            comments[len(lines)] = comment_summarizer(field.comment)
                        lines.append(s)
                for (i, line) in enumerate(lines):
                    ss += f"{line}"
                    if i in comments:
                        ss += " " * (max_len - len(line))
                        ss += " # " + comments[i]
                    ss += "\n"
        if pkg.srvs:
            for srv in pkg.srvs:
                s = f"  - srv: {srv.srv}"
                dsc = description_summarizer(srv.description)
                if dsc:
                    s += f" # {dsc}"
                s += "\n"
                ss += s

                max_len = 0
                lines = []
                comments = {}
                if srv.requests:
                    for field in srv.requests:
                        s = f"    - (req) {field.name}: {field.type_}"
                        if field.default_value:
                            s += f" (={field.default_value})"
                        if len(s) > max_len:
                            max_len = len(s)
                        if field.comment:
                            comments[len(lines)] = comment_summarizer(field.comment)
                        lines.append(s)
                for (i, line) in enumerate(lines):
                    ss += f"{line}"
                    if i in comments:
                        ss += " " * (max_len - len(line))
                        ss += " # " + comments[i]
                    ss += "\n"

                max_len = 0
                lines = []
                comments = {}
                if srv.responses:
                    for field in srv.responses:
                        s = f"    - (res) {field.name}: {field.type_}"
                        if field.default_value:
                            s += f" (={field.default_value})"
                        if len(s) > max_len:
                            max_len = len(s)
                        if field.comment:
                            comments[len(lines)] = comment_summarizer(field.comment)
                        lines.append(s)
                for (i, line) in enumerate(lines):
                    ss += f"{line}"
                    if i in comments:
                        ss += " " * (max_len - len(line))
                        ss += " # " + comments[i]
                        ss += "\n"
        if pkg.actions:
            for action in pkg.actions:
                s = f"  - action: {action.action}"
                dsc = description_summarizer(action.description)
                if dsc:
                    s += f" # {dsc}"
                s += "\n"
                ss += s

                max_len = 0
                lines = []
                comments = {}
                if action.goal_requests:
                    for field in action.goal_requests:
                        s = f"    - (goal) {field.name}: {field.type_}"
                        if field.default_value:
                            s += f" (={field.default_value})"
                        if len(s) > max_len:
                            max_len = len(s)
                        if field.comment:
                            comments[len(lines)] = comment_summarizer(field.comment)
                        lines.append(s)
                for (i, line) in enumerate(lines):
                    ss += f"{line}"
                    if i in comments:
                        ss += " " * (max_len - len(line))
                        ss += " # " + comments[i]
                        ss += "\n"
                
                max_len = 0
                lines = []
                comments = {}
                if action.responses:
                    for field in action.responses:
                        s = f"    - (res) {field.name}: {field.type_}"
                        if field.default_value:
                            s += f" (={field.default_value})"
                        if len(s) > max_len:
                            max_len = len(s)
                        if field.comment:
                            comments[len(lines)] = comment_summarizer(field.comment)
                        lines.append(s)
                for (i, line) in enumerate(lines):
                    ss += f"{line}"
                    if i in comments:
                        ss += " " * (max_len - len(line))
                        ss += " # " + comments[i]
                        ss += "\n"

                max_len = 0
                lines = []
                comments = {}
                if action.feedback:
                    for field in action.feedback:
                        s = f"    - (feedback) {field.name}: {field.type_}"
                        if field.default_value:
                            s += f" (={field.default_value})"
                        if len(s) > max_len:
                            max_len = len(s)
                        if field.comment:
                            comments[len(lines)] = comment_summarizer(field.comment)
                        lines.append(s)
                for (i, line) in enumerate(lines):
                    ss += f"{line}"
                    if i in comments:
                        ss += " " * (max_len - len(line))
                        ss += " # " + comments[i]
                        ss += "\n"
    touch @(output_dist)
    with open(output_dist, "w") as f:
        f.write(ss)

COMMON_INTERFACES_PATH = "../src/ros2/common_interfaces"
COMMON_INTERFACES_MSGS_YAML_OUTPUT_PATH = "../msgs/common_interfaces_msgs.yaml"
COMMON_INTERFACES_MSGS_MD_OUTPUT_PATH = "../msgs/common_interfaces_msgs.md"
COMMON_INTERFACES_MSGS_TXT_OUTPUT_PATH = "../msgs/common_interfaces_msgs.txt"
AUTOWARE_MSGS_PATH = "../autoware_msgs"
AUTOWARE_MSGS_MSGS_YAML_OUTPUT_PATH = "../msgs/autoware_msgs_msgs.yaml"
AUTOWARE_MSGS_MSGS_MD_OUTPUT_PATH = "../msgs/autoware_msgs_msgs.md"
AUTOWARE_MSGS_MSGS_TXT_OUTPUT_PATH = "../msgs/autoware_msgs_msgs.txt"

process_msgs_universe(COMMON_INTERFACES_PATH, COMMON_INTERFACES_MSGS_YAML_OUTPUT_PATH)
process_msgs_universe(AUTOWARE_MSGS_PATH, AUTOWARE_MSGS_MSGS_YAML_OUTPUT_PATH)

summary(COMMON_INTERFACES_PATH, COMMON_INTERFACES_MSGS_MD_OUTPUT_PATH)
summary(AUTOWARE_MSGS_PATH, AUTOWARE_MSGS_MSGS_MD_OUTPUT_PATH)

txt_summary(COMMON_INTERFACES_PATH, COMMON_INTERFACES_MSGS_TXT_OUTPUT_PATH)
txt_summary(AUTOWARE_MSGS_PATH, AUTOWARE_MSGS_MSGS_TXT_OUTPUT_PATH)