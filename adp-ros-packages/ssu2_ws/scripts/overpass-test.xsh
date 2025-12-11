#!/usr/bin/env xonsh

import requests
import json

url = "https://overpass-api.de/api/interpreter"

query = """
[out:json];
(
  node["highway"="traffic_signals"]({{bbox}});
  way["highway"="traffic_signals"]({{bbox}});
  relation["highway"="traffic_signals"]({{bbox}});
);