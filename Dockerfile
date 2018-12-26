FROM ros:kinetic-ros-core

# Install rospy
RUN apt-get update
RUN apt-get install -y ros-kinetic-rospy ros-kinetic-rviz
RUN apt-get install -y python-pip python-numpy python-scipy

RUN pip install --upgrade pip
RUN pip install PyGeometry

# RUN apt-get update && \
#    apt-get upgrade -y && \
RUN apt-get install -y git

# RUN mkdir -p ~/.ssh
# RUN ssh-keyscan -H github.com >> ~/.ssh/known_hosts
# RUN ssh git@github.com git-lfs-authenticate "${CIRCLE_PROJECT_USERNAME}/${CIRCLE_PROJECT_REPONAME}" download
# RUN git lfs pull

RUN cd ~
# Set up prerequisites
# The sed command is used to remove the first line of requirements.txt which is PyYAML because it stops the execution of the installation midway and dependencies still remain
RUN git clone https://github.com/duckietown/duckietown-world.git ~/duckietown-world; cd ~/duckietown-world; sed '2d' requirements.txt > requirements1.txt; pip install -r requirements1.txt; python setup.py develop --no-deps

RUN cd ~
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash
RUN apt-get update
RUN apt-get install -y git-lfs openssh-client
RUN git lfs install
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash"
# Setup duckietown uplan
RUN mkdir -p ~/catkin_ws/src
#; cd
RUN cd ~/catkin_ws/src
RUN git clone https://github.com/duckietown/duckietown-uplan.git ~/catkin_ws/src/duckietown-uplan; cd ~/catkin_ws/src/duckietown-uplan/lib-uplan; pip install -r requirements.txt --user; python setup.py develop --no-deps

RUN pip install networkx==2.2
# RUN touch requirements.txt
# COPY /home/aroumie/catkin_ws/src/duckietown-uplan/lib-uplan/requirements.txt .
# RUN pip install -r requirements.txt --user
# RUN python setup.py develop --no-deps

RUN cd ~
ENV DISPLAY $DISPLAY
ENV QT_X11_NO_MITSHM 1

# Setup ros
RUN cd ~
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd ~/catkin_ws; catkin_make; source ~/catkin_ws/devel/setup.bash"

CMD /bin/bash -c "cd ~; source /opt/ros/kinetic/setup.bash; cd ~/catkin_ws; source ~/catkin_ws/devel/setup.bash; roslaunch uplan_visualization planningUncertainty.launch"
#; roslaunch uplan_visualization planningUncertainty.launch"
#; roslaunch uplan_visualization planningUncertainty.launch"

# Run the visualization
# RUN /bin/bash -c "source devel/setup.bash"
# RUN roslaunch uplan_visualization planningUncertainty.launch

# RUN /bin/bash -c "cd /catkin-ws; source /catkin-ws/devel/setup.bash; roslaunch /catkin-ws/src/duckietown-uplan/ros-uplan/uplan_visualization/launch planningUncertainty.launch"
