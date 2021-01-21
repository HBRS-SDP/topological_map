#!/usr/bin/env python

from setuptools import setup

setup(
    name="topological_map",
    packages=[
        "topological_map",
        "topological_map.common.map_utils",
        "topological_map.maps",
        "topological_map.maps.*",
    ],
    version="1.0.0",
    description="BRSU Topological Map",
    package_dir={"": "."},
    package_data={"": ["*.yaml", "*.pgm"]},
)
