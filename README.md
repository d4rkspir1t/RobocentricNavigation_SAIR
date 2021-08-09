## RobocentricNavigation_SAIR ## 

The project makes use of the ROS# repository from here: https://github.com/siemens/ros-sharp

This repository contains the script "listener.py", which runs a subscriber node to receive Transform values, in the form of "PoseStamped" data type.

# Requirements
    - Ubuntu.
    - [ROS framework](http://wiki.ros.org/).
    - [ROS_Bridge server package](http://wiki.ros.org/rosbridge_server?distro=noetic).
    - Unity3D.

# Steps to run:
    1. Download and extract zip folder from [ROS# repository](https://github.com/siemens/ros-sharp).

    2. Add the RosSharp folder into Unity Assets in (Unity3D/Assets/RosSharp).

    3. In Unity's scene, add an empty object and call it: "RosConnector", we will add components here.

    4. Add a script component "Ros Connector", set the "Ros Bridge server URL" to ROS socket server.

    5. Add a script component "Pose stamped publisher", choose the topic to publish, and choose transform.

    6. Run the scene, and the publisher script in step (5) will start publishing to the topic. 

    7. Run the "listener.py" node to receive the values and display them.