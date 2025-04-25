
# ROS Behavior Tree Corrosion Mapping Node

## Overview

This ROS package implements a node (`trigger_servo_node`) that utilizes the `behaviortree_cpp_v3` library to execute a predefined sequence of actions for controlling robotic components. The behavior tree orchestrates triggering a servo motor, adjusting a joint angle, and introducing timed delays. The node also provides a service to gracefully abort its execution.

The default behavior tree executes the following sequence repeatedly:
1.  **TriggerServo**: Sends a command to a linear motor/servo.
2.  **Sleep**: Pauses execution for a duration dynamically received from a topic.
3.  **TriggerJointAngle**: Calls a service to set a specific joint angle.
4.  **SleepNode**: Pauses execution for a fixed duration defined in the Behavior Tree XML.

## Dependencies

* **ROS** (Tested on Noetic, likely compatible with Melodic and newer)
    * `roscpp`
    * `std_msgs`
    * `std_srvs`
* **BehaviorTree.CPP V3** (`behaviortree_cpp_v3`): The core library for creating and running behavior trees.
* **C++11 (or later) Compiler**
* **Catkin** (or `colcon` for ROS2-style builds if adapted)

## Building

This package is designed to be built within a standard ROS Catkin workspace.

1.  **Build the Workspace:**
    ```bash
    cd ~/catkin_ws
    catkin build
    ```

2.  **Source the Workspace:**
    ```bash
    source ~/catkin_ws/devel/setup.bash
    ```

## Usage

1.  **Ensure ROS Master is Running:**
    ```bash
    roscore
    ```

2.  **Launch Prerequisite Nodes (if any):**
    * Make sure the node providing the `/set_joint_angle` service is running.
    * Make sure the node subscribing to the `/linear_motor` topic is running.
    * Optionally, have a node publishing the desired sleep duration to `/rotation_time`.

3.  **Run the Node:**
    ```bash
    rosrun corrosion_mapping trigger_servo_node
    ```

The node will start executing the behavior tree defined internally.

## Node Details (`trigger_servo_node`)

### Published Topics

* **/linear\_motor** (`std_msgs/Int32`)
    * Publishes commands intended for a linear actuator or servo. Alternates between `1` and `2` on subsequent calls from the `TriggerServo` BT node.

### Subscribed Topics

* **/rotation\_time** (`std_msgs/Float32`)
    * Listens for float values representing sleep duration in seconds. This dynamically updates the pause duration used by the `Sleep` BT node. Defaults to 1.0 second if no messages are received.

### Services Provided

* **/abort\_raster\_scanning** (`std_srvs/SetBool`)
    * Allows external control to stop the node's execution gracefully. Calling the service with `data: true` will trigger a `SIGABRT`, terminating the node via its signal handler.
    * **Request:** `bool data` - Set to `true` to abort.
    * **Response:** `bool success`, `string message`

### Services Called

* **/set\_joint\_angle** (`std_srvs/Trigger`)
    * Calls this service to trigger an action, presumably setting a predefined joint angle on a robot.
    * The `TriggerJointAngle` BT node calls this service. Success/failure of the service call determines the success/failure of the BT node tick.

### Behavior Tree Structure

The node executes a Behavior Tree defined internally as an XML string.

**Default Tree Structure:**

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <TriggerServo linear_motor="{linear_motor}"/>
      <Sleep/>
      <TriggerJointAngle/>
      <SleepNode duration="1"/>
    </Sequence>
  </BehaviorTree>
</root>
```

**Custom Behavior Tree Nodes:**

* **`TriggerServo`** (`BT::SyncActionNode`)
    * **Description:** Publishes an alternating integer (`1` or `2`) to the topic specified by the `linear_motor` input port.
    * **Input Port:** `linear_motor` (Type: `ros::Publisher`) - Expects the ROS publisher handle for `/linear_motor` passed via the blackboard.
    * **Returns:** `SUCCESS` after publishing.

* **`Sleep`** (`BT::SyncActionNode`)
    * **Description:** Pauses the execution thread for a duration. The duration is dynamically updated by subscribing to the `/rotation_time` topic.
    * **Returns:** `SUCCESS` after the pause.

* **`TriggerJointAngle`** (`BT::SyncActionNode`)
    * **Description:** Calls the `/set_joint_angle` ROS service (`std_srvs/Trigger`).
    * **Returns:** `SUCCESS` if the service call is successful and the service response indicates success. `FAILURE` otherwise.

* **`SleepNode`** (`BT::SyncActionNode`)
    * **Description:** Pauses the execution thread for a fixed duration specified in the BT XML.
    * **Input Port:** `duration` (Type: `int`) - The sleep duration in seconds.
    * **Returns:** `SUCCESS` after the pause.

### Aborting Execution

To stop the node externally, you can call the `/abort_raster_scanning` service:

```bash
rosservice call /abort_raster_scanning "data: true"
```

This will cause the node to print a warning, set the service response, and then raise `SIGABRT`, leading to termination via the registered signal handler.

## Contributing
Nilanjan Chowdhury
