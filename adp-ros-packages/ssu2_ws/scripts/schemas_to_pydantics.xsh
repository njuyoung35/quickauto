#!/usr/bin/env xonsh

schemas_path = "../schemas"
schemas = [
    "universe.schema.yaml",
    "meta_config.schema.yaml"
]

for schema in schemas:
    spath = f"{schemas_path}/{schema}"
    out_path = f"{schema.split('.')[0]}.py"
    datamodel-codegen --input @(spath) --output @(out_path) --input-file-type jsonschema