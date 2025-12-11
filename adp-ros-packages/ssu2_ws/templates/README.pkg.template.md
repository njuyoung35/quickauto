{%- macro get_msgs_name(input) -%}
{{- input.msg or input.srv or input.action -}}
{%- endmacro -%}

{%- macro get_msgs_type(input) -%}
{%- if input.msg is not none -%}msg
{%- elif input.srv is not none -%}srv
{%- elif input.action is not none -%}action
{%- endif -%}
{%- endmacro -%}

# {{ pkg.pkg }}

pkg_type: {{ pkg.type.value }}

{% if pkg.purpose is not none -%}
## Purpose

{{ pkg.purpose | trim }}
{%- endif %}
{% if pkg.design is not none -%}
## Design

{{ pkg.design }}
{%- endif %}
{% if pkg.inputs is not none -%}
## Inputs

{% for input in pkg.inputs -%}
  {%- set msgs_name = get_msgs_name(input) -%}
  {%- set msgs_type = get_msgs_type(input) -%}
- {{ msgs_name }}: `({{ input.type }})`
  {% for desc_line in input.description.split('\n') %}
  > {{ desc_line }} \
  {%- endfor %}
{%- endfor %}
{%- endif %}

{% if pkg.outputs is not none -%}
## Outputs

{% for output in pkg.outputs -%}
  {%- set msgs_name = get_msgs_name(output) -%}
  {%- set msgs_type = get_msgs_type(output) -%}
- {{ msgs_name }}: `({{ output.type }})`
  {%- for desc_line in output.description.split('\n') -%}
  > {{ desc_line }} \
  {%- endfor %}
{%- endfor %}
{%- endif %}

{% if pkg.references is not none -%}
## References
{% for ref in pkg.references %}
- {{ ref }}
{% endfor %}
{%- endif %}