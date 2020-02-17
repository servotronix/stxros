# STX-Controller: ER9 

##**Introduction**


 The STX-Controller will allow us to move and communicate with out robots 
 by writing or changing the command node 'moveit_node' or by moving the robot in the 'Rviz' simulation.
 
 The communication is throw a TCP-Telnet/UDP protocols, in which the MC is the server and 
 the other nodes: 'arm_controller', 'stx_joint_state_publisher' and 
 'moveit_node' are on the client side.
 
##**Starting the system**


 In order to start the robot movement we need to execute the following in terminal:
 
 - first clone the repository from: 
 
        /git address 

 - open the project amd build the work space by writing the command:
 
        catkin_make 
 
 - after cloning the files and building the catkin workspace make sure to be in the workspace
  directory by executing:
 
        cd ~/er9-ros-workspace


 - Staring the ROS system with the er9 robot connected:
 
        roslaunch stx_control run_er9.launch 
 
 - In case robot is not connected we provide a 'simulation' mod which can be started by executing:
 
        roslaunch stx_control simulation.launch
      
  *optional:
  
 - Adding the Moveit! controller node to our system (from a new terminal window):
 
        rosrun stx_control moveit_node.py
 
  
  **The system is up and ready to take commands!**
   
 
 
 ##**Operating the system**
 
 
   In order to plan a new robot movement the moveit node needs to be edited.
   
   The 'moveit_node.py' node shows basic robot movement which can be executed by pressing the 
   keys instructed on the moveit_node.py terminal consul window.
   
-   **The node could be edited or replaced in order to move the robot in a different way.**
   
   
   
 ##**Homing procedure**
 
If the robot has been turn off during movement execution or suffered from a different kind of unexpected
error and now is not compatible with the 'Rviz' simulation we wil need to do manual 'Homing'.

 - execute in a new terminal window the command (make sure to be in er9-ros-workspace directory):
 
        python src/stx_control/scripts/GUI_homing.py
  
  When the controller appears you will need to move each joint to the 'Home' position and after press 'Set joint!.
 
 Once all the joints are set we can quit the homing controller and start planing.



    
  ####_**Additional control tips:**_

- _Gripper control:_

   In order to control gripper from 'Rviz', go to 'Planing Request' and
 change the planing group label to 'gripper'. 
 
   Under 'Planing' label set the 'Select goal state' to be the wanted gripper position.
   After, press the 'Update' button and you are ready to execute the command with 'Plan and Execute'
   button.
 
 
- _Exit the system:_

    press 'Ctrl + c' on terminal window.

