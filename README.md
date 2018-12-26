[![CircleCI](https://circleci.com/gh/aroumie1997/duckietown-uplan.svg?style=shield)](https://circleci.com/gh/aroumie1997/duckietown-uplan)

[![Coverage Status](https://coveralls.io/repos/github/aroumie1997/duckietown-uplan/badge.svg?branch=master18)](https://coveralls.io/github/aroumie1997/duckietown-uplan?branch=master18)

[![PyPI status](https://img.shields.io/pypi/status/duckietown-uplan.svg)](https://pypi.python.org/pypi/duckietown-uplan/)


[![PyPI pyversions](https://img.shields.io/pypi/pyversions/duckietown-uplan.svg)](https://pypi.python.org/pypi/duckietown-uplan/)


# Planning with Uncertainty

This project aims to plan a path for a duckiebot while taking into account various sources of uncertainty.

## Pre-flight checklist {#demo-planningunderuncertainty-pre-flight}

duckietown-world


Those following prerequisites ensure that the simulation will run properly:

Check: Desktop-Full installation of [ros-kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

Check: [duckietown-world](https://github.com/duckietown/duckietown-world) (And all their requirements)

Check: Ubuntu16.04 with python2.7


## Demo instructions {#demo-planningunderuncertainty-run}

Here, give step by step instructions to reproduce the demo.

** ASSUMTION: YOU ARE ABLE TO IMPORT `duckietown_world` SUCCESFULLY **

Step 0: Make sure you sourced ros

    $ source /opt/ros/kinetic/setup.bash       (Use setup.zsh If you are using zsh shell)

Step 1: Create a catkin workspace in your home directory

    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
  
Step 2: Clone the following repo [duckietown-uplan](https://github.com/duckietown/duckietown-uplan) 

    $ git clone https://github.com/duckietown/duckietown-uplan

Step 3: Installation of dependencies (NOTE: make sure that pip installation belongs to python2.7)

    $ cd duckietown-uplan/lib-uplan
    $ pip install -r requirements.txt --user       (Might require --user)
    $ sudo python setup.py develop --no-deps
    
Step 4: Go back to catkin_ws main directory and run catkin_make

    $ cd ../../..
    $ catkin_make
    
Ensure all three packages build correctly and custom messages are built, now two more steps left:

    $ source devel/setup.bash               (Or zsh)
    $ roslaunch uplan_visualization planningUncertainty.launch
