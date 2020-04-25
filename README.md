# Roberto IK Planner

## Instructions to Run
1. Clone the repository into the src directory of your catkin workspace.
2. Make sure the `tiago_gazebo` directory is in the catkin workspace (it comes with the tiago installation).
2.  Build catkin using either `catkin build roberto_ik_planner` or `catkin_make` (not sure if the second will work).
3. Open the robot in gazebo with 

    ```
    $ roslaunch roberto_ik_planner pick_simulation.launch
    ```
    
    This will launch the gazebo server using the launch file in `/launch`. It contains the robot with a small block in front of it.
4. In a new terminal, run the pickup client node using
    ```
    $ roslaunch roberto_ik_planner pick_demo.launch
    ```

    This starts the pick and place server, which handles the actual pickup requests, and the pickup client, which handles receiving the goals on a ros topic and processing it before sending it to the pickup server.

5. Send a goal to the client using
    ```
    $ rosrun roberto_ik_planner talker.py
    ```

    in a new terminal. The talker will construct a `PoseStamped` object (currently hardcoded to the position of the object in the world) and broadcast it on the `pick_goals` topic that `pickup.py` is subscribed to. You should see "Goal received" in the terminal window where step 4 was executed. The robot will then plan a grasp to that object and attempt to pick it up. In RViz, you should see the possible grasps show up as a vector field around the object.


5. Place the object somewhere using
    ```
    $ rosrun roberto_ik_planner publish_place.py
    ```

    once the robot has successfully picked up the object. ```publish_place.py``` publishes a PoseStamped object which is the pose of the desired location to drop the object. Once the arm moves to the correct location, the gripper opens and the object should fall out. 


## Relevant Files
- /scripts/ik_planner.py: Ros node that handles main driver for just IK movement of end effector. Receives goal poses on topic `pose_goals` and moves end effector to that pose.
- /scripts/pickup.py: Main client of picking up objects with grasp capability. Recieves goals on topic `pick_goals` and picks up the object at that position.
- /scripts/spherical_grasps_server.py: Generates possible grasps for a given pose, used by pick and place server. Copied from tiago pick and place tutorial.
- /scripts/pick_and_place_server.py: Server for pick and place motions. Given a pose, generates possible grasps and executes the best grasp. Copied from tiago pick and place tutorial.
- /scripts/talker.py: Sends pickup goal to pickup client on `pick_goals` as a PoseStamped object.


## TODOs
- Ensure this still works if the robot is not at the origin but has moved somewhere else. In theory I think the code should work fine, as long as the pose of the object is correct. If the object pose is in the robot frame, we will need to convert it to the world frame.
- Try picking up multiple objects?
- Haven't tried placing objects yet

## Useful links
[Tiago tutorials](http://wiki.ros.org/Robots/TIAGo/Tutorials/MoveIt/Pick_place)

[MoveIt tutorials](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html)