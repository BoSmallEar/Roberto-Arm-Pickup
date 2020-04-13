# Roberto IK Planner

## Relevant Files
- /scripts/ik_planner.py: Ros node that handles main driver. Receives goal poses on topic `pose_goals` and moves end effector to that pose. Currently only capable of moving arm, not gripper.
- /scripts/pickup.py: Test script to pick up objects using gripper. Currently does not work.
- /scripts/ik_planner.py: ros node. Receives goal poses on topic `pose_goals` and moves end effector to that pose. Currently only capable of moving arm, not gripper.


## Instructions to Run
1. Clone the repository into the src directory of your catkin workspace.
2.  Build catkin using either `catkin build roberto_ik_planner` or `catkin_make` (not sure if the second will work).
3. Open the robot in gazebo with 

    ```
    $ roslaunch roberto_ik_planner roberto_ik_plan.launch
    ```
    
    This will launch the gazebo server using the launch file in `/launch`. It contains the robot in an empty world, and opens the gazebo client and RViz for visualization as well. If you just want the server without gazebo or RViz (e.g. for debugging), use roberto_server_only.launch instead.
4. In a new terminal, run the driver node using
    ```
    $ rosrun roberto_ik_planner ik_planner.py
    ```

    This will start the `ik_planner.py` script as a ros node.

5. Send a goal to `ik_planner.py` using
    ```
    $ rosrun roberto_ik_planner talker.py 0.4 -0.3 0.26 0 0 0
    ```

    in a new terminal. The talker will construct a `Pose` object from the command-line arguments (first three are xyz position, last three are roll-pitch-yaw orientation) and broadcast it on the `pose_goals` topic that `ik_planner.py` is subscribed to. You should see "goal received" in the terminal window where `ik_planner.py` is running. The robot will then plan how to move the arm to that given pose and broadcast it on the `/move_group/display_planned_path` topic for RViz to visualize, before actually carrying out the motion.

6. If the path does not come up in RViz, you may have to click on "Add" on the left hand panel and click on "Motion Planning" (this sometimes works and sometimes doesn't, not sure why yet).

## TODOs
- Figure out how to manipulate the gripper. Currently following the [Tiago pick and place tutorial](http://wiki.ros.org/Robots/TIAGo/Tutorials/MoveIt/Pick_place), but that uses a lot of things we don't need like object detection with OpenCV, so I need to figure out what components I can remove and how to adapt the remainder for our purpose.
- Create a world with objects in it that we can try picking up.

## Useful links
[Tiago tutorials](http://wiki.ros.org/Robots/TIAGo/Tutorials/MoveIt/Pick_place)

[MoveIt tutorials](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html)