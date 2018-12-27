[![CircleCI](https://circleci.com/gh/duckietown/duckietown-uplan.svg?style=shield)](https://circleci.com/gh/duckietown/duckietown-uplan)

[![Coverage Status](https://coveralls.io/repos/github/duckietown/duckietown-uplan/badge.svg?branch=master18&service=github)](https://coveralls.io/github/duckietown/duckietown-uplan?branch=master18)

[![PyPI status](https://img.shields.io/pypi/status/duckietown-uplan.svg)](https://pypi.python.org/pypi/duckietown-uplan/)

[![PyPI pyversions](https://img.shields.io/pypi/pyversions/duckietown-uplan.svg)](https://pypi.python.org/pypi/duckietown-uplan/)

# Planning with Uncertainty

This project aims to plan a path for a duckiebot while taking into account various sources of uncertainty. Upon successful completion of the demo, you should expect a simulation implementing both collision-free path planning and velocity profiling algorithms, as in the figure below:

<div figure-id="fig:expected_results" figure-caption="Expected Result when running the simulation demo">
     <img src="/resources/Picture1.png" style='width: 30em'/>
</div>

## Pre-flight checklist 

Those following prerequisites ensure that the simulation will run properly:

Check: Desktop-Full installation of [ros-kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

Check: [duckietown-world](https://github.com/duckietown/duckietown-world) (and all its requirements)

Check: Ubuntu16.04 with python2.7

Check: [Docker installation](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-docker-ce) for docker instructions


## Demo instructions 

### Docker instructions

First, pull the docker image:

    $ docker pull aroumie1997/uplan:v1
    
Next, run:

```
$ docker run -it \
   --env="DISPLAY" \
   --env="QT_X11_NO_MITSHM=1" \
   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
   aroumie1997/uplan:v1

```

**Note:** In order to see the rviz simulation, the host OS should have a desktop-full installation of ros with rviz. You may need to run the following line before running the container:

    $ xhost +local:root
    
After running the container, do not forget to run the following, as this compromises access control to X server on your host:

    $ xhost -local:root

### ROS-only instructions

**ASSUMPTION: YOU ARE ABLE TO IMPORT `duckietown_world` SUCCESSFULLY**

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
