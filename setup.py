#!/usr/bin/env python

from setuptools import setup

setup(
    name="topological_map",
    packages=[
        "topological_map_server",
        "topological_map_server.common.map_utils",
        "topological_map_server.maps",
        "topological_map_server.maps.brsu-full",
        "topological_map_server.maps.brsu-large",
        "topological_map_server.maps.brsu-osm",
        "topological_map_server.maps.brsu-small",
        "topological_map_server.maps.brsu-small-free",
        "topological_map_server.maps.brsu-small-obs",
    ],
    version="1.0.0",
    description="BRSU Topological Map",
    package_dir={"": "."},
    package_data={"": ["*.yaml", "*.pgm"]},
)
