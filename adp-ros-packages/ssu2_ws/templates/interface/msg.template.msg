{% if msg.description %}
{{ msg.description | comment_hash }}
{% endif %}

{% for field in msg.fields %}
{% if field.description %}
{{ field.description | comment_hash }}
{% endif %}
{{ field.type }} {{ field.name }}
{% endfor %}