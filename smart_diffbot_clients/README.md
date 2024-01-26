# AAN navigation clients

Package containing simple examples of Python ROS 2 clients to send instructions to the smart_diffbot robot created for the Autonomous Agricultural Navigation project. Running one of these clients will result in instructions (action goals) being sent to the Nav2 stack of the smart_diffbot running in simulation. 

## Field coverage
Field coverage can be started through two types of clients:

### Field cover client with planning
This client includes the coverage path planning intelligence, meaning it will just send the resulting waypoints to the robot's Nav2 stack. It gets picked up by the ExactPathNavigator of the robot through the ```/navigate_exact_path``` action server. 

```bash
ros2 run smart_diffbot_clients field_cover_client_w_planning 
```

This is a kind of 'dumb robot - smart dispatcher' example. 

### Field cover client without planning
!!!TBD!!! <!-- This client sends the field and task characteristics to the FieldCoverageNavigator of the robot through the ```/cover_field``` action server. The coverage path planning is then ran inside of the robot's Nav2 stack as a planner plugin. -->

<!-- >```bash
ros2 run smart_diffbot_clients field_cover_client_no_planning
``` -->

<!-- This ia a kind of 'smart robot - dumb dispatcher' example.  -->

## Row following
!!!TBD!!!

## Docking 
Docking can be achieved by running the docking client:

```bash
ros2 run smart_diffbot_clients docking_client
```

It sends the approximate approach location of the docking station to the Nav2 stack of the smart_diffbot, which will approach it, look for the marker and finally dock. 