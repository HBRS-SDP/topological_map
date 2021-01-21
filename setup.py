#!/usr/bin/env python

from setuptools import setup

setup(
    name="topological_map",
    packages=[
        "topological_map",
        "topological_map.common.map_utils",
        "topological_map.maps",
        "topological_map.maps.brsu-full",
        "topological_map.maps.brsu-large",
        "topological_map.maps.brsu-osm",
        "topological_map.maps.brsu-small",
        "topological_map.maps.brsu-small-free",
        "topological_map.maps.brsu-small-obs",
    ],
    version="1.0.0",
    description="BRSU Topological Map",
    package_dir={"": "."},
)
