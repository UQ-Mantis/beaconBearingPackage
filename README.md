# beaconBearingPackage
This package records audio from hydrophones through an audio recording device and calculates the bearing to an underwater beacon 
Used with ros noetic
Programs are written in Python

Raspberry pi 
password: Robotx1

Use:
1. source termial
    - source /opt/ros/noetic/setup.bash
    - source devel/setup.bash
2. cd catkin_ws
3. have a roscore running in another terminal

Recording
5. rosrun py_bearing python_filtering.py
    issues: 
     - does not connect to audio device:
          - retry command until it works
          -  restart audio device
          -  check audio input is correct (alsamixer to check what port audio device is on, change port number in code)

Filtering and bearing calcualtion
rosrun py_bearing filtering.py
