[![CircleCI](https://circleci.com/gh/duckietown/duckietown-uplan.svg?style=shield)](https://circleci.com/gh/duckietown/duckietown-uplan)

[![Coverage Status](https://coveralls.io/repos/github/duckietown/duckietown-uplan/badge.svg?branch=master18)](https://coveralls.io/github/duckietown/duckietown-uplan?branch=master18)

[![PyPI status](https://img.shields.io/pypi/status/duckietown-uplan.svg)](https://pypi.python.org/pypi/duckietown-uplan/)


[![PyPI pyversions](https://img.shields.io/pypi/pyversions/duckietown-uplan.svg)](https://pypi.python.org/pypi/duckietown-uplan/)


# Planning with Uncertainty

This project aims to plan a path for a duckiebot while taking into account various sources of uncertainty.


## Installation from source

This is the way to install within a virtual environment created by 
using `pipenv`:

    $ pipenv install
    $ pipenv shell
    $ cd lib-uplan
    $ pip install -r requirements.txt
    $ python setup.py develop --no-deps

## Pre-check
* Ubuntu16.04 with python2.7
* Desktop-Full installation of ROS Kinetic
* duckietown-world

## Installation
    $ git clone https://github.com/duckietown/duckietown-uplan.git 

Create a ROS workspace add symbolic links using 

    $ ln -s <path to source> <path to target>
    
Link to uplan_visualization and duckietown_visualization packages in duckietown-uplan/ros-plan from /src folder in your ROS workspace

Add symbolic links from duckietown_uplan in lib-uplan folder to your workspace

Installation: cd/src/uncertainty_planning/src/duckietown-uplan/lib-uplan

    $ pip install -r requirements.txt
    $ catkin_make 

and ensure all three packages build correctly and custom messages are built
    
    $ source devel/setup.bash
    $ roslaunch uplan_visualization planningUncertainty.launch
