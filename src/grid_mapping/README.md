
# ROS Behavior Tree Node for Automated Task Sequence

This ROS package implements a node that utilizes the BehaviorTree.CPP v3 library to orchestrate a sequence of actions involving linear actuators (LAC), a pump, an electromagnet, and a slider mechanism. It's designed for controlling automated tasks, potentially in a testing, manufacturing, or robotic manipulation context.

The behavior tree logic is defined within the C++ code and executed in a ROS environment.

## Dependencies

* **ROS:** (Specify your distribution, e.g., Noetic, Melodic)
* **BehaviorTree.CPP v3:** The core library for creating and running behavior trees. ([https://github.com/BehaviorTree/BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP))
* **catkin:** Standard ROS build system.
* **ROS Standard Messages:** `std_msgs`, `geometry_msgs`, `sensor_msgs`.
* **ROS Standard Services:** `std_srvs` (specifically `Trigger`, `SetBool`).
* **Custom ROS Messages/Services:**
    * `stm_interface::RelayControl`: Assumed to be a custom service definition for toggling relays. You need this package in your workspace.

## Building

1.  Clone this repository into your catkin workspace's `src` directory.
    ```bash
    cd ~/catkin_ws/src
    ```
2.  Ensure you have all dependencies installed, especially `behaviortree_cpp_v3` and the `stm_interface` package.
3.  Build the workspace:
    ```bash
    catkin build
    # or catkin build if you use catkin tools
    source devel/setup.bash
    ```

## Running the Node

Launch the node using `rosrun`:

```bash
rosrun grid_mapping bt_lac_zero_node
```

## Functionality

This node executes a predefined Behavior Tree.

### Behavior Tree Structure

The core logic is defined by the following sequence hardcoded in the `main` function:

```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <Electromagnet_On/>
            <LAC_Down/>
            <Sleep duration="5"/>
            <Electromagnet_Off/>
            <LAC_Zero/>
            <Slider_shift/>
        </Sequence>
    </BehaviorTree>
</root>
```

This sequence performs the following steps repeatedly:
1.  `Electromagnet_On`: Activates an electromagnet and increments/publishes a grid counter.
2.  `LAC_Down`: Moves the linear actuator (LAC) down.
3.  `Sleep`: Pauses execution for 5 seconds.
4.  `Electromagnet_Off`: Deactivates the electromagnet.
5.  `LAC_Zero`: Moves the linear actuator (LAC) to a zero/home position and potentially interacts with a linear motor based on cycle counts and stroke length input.
6.  `Slider_shift`: Performs a conditional action. It counts cycles and, upon reaching a limit (`cycle_limit_`), multiplies a stroke length value (`stroke_length_value`), publishes it, moves a linear motor/servo, triggers a joint angle setting service (`/set_joint_angle`), waits, and then publishes the original stroke length. If the cycle limit isn't reached, it fails, likely causing the parent sequence node (if part of a larger tree structure not shown here) or the loop to handle the failure/repetition.

### Custom Behavior Tree Nodes

The following custom BT nodes are implemented:

* **`LAC_Zero`**: Publishes `1` then `0` to `/motor_direction` . Publishes `1` to `/linear_motor` . Subscribes to `/ui_stroke_length_publisher` and `/cycles_publisher` to modify its behavior based on UI input and cycle counts (skipping linear motor activation after `cycle_limit_`). Includes delays.
* **`Pump`**: Calls the `/relay_toggle_channel` service twice with data `3`, toggling a pump connected to relay channel 3. Includes a delay between calls.
* **`Electromagnet_On`**: Calls the `/relay_toggle_channel` service with data `5` (turning *on* an electromagnet). It also increments a counter (`gridCounter_`), publishes it to `/grid_number`, and resets it based on `maxGridNum` (derived from `/cycles_publisher`).
* **`Sleep`**: A simple synchronous action that pauses execution for a specified duration (in seconds) provided as an input port `duration`.
* **`LAC_Down`**: Publishes `2` to `/motor_direction`. Calls `/relay_toggle_channel` service with data `3`. Includes delays. Subscribes to `/ui_stroke_length_publisher`.
* **`Electromagnet_Off`**: Calls the `/relay_toggle_channel` service with data `5` (turning *off* the electromagnet).
* **`Dynamic_Sleep`**: Sleeps for a duration dynamically received from the `/rotation_time` topic (`std_msgs::Float32`).
* **`Slider_shift`**: A stateful node. It counts ticks. If the count is below `cycle_limit_` (received from `/cycles_publisher`), it returns `FAILURE`. Once the limit is reached, it reads `/stroke_length_publisher`, multiplies the value by `multiplier_` (from `/multiplier_publisher`), publishes the result to `/stroke_length`, triggers `/linear_motor` (data `2`), sleeps for a duration calculated based on UI stroke length and multiplier, calls the `/set_joint_angle` service, waits based on `/set_joint_angle_publisher` data, publishes the *original* stroke length back to `/stroke_length`, resets its counter, and returns `SUCCESS`.

### Control Features

* **Pause/Resume:** The node advertises a `/pause_tree` service (`std_srvs::Trigger`). Calling this service toggles the execution state of the behavior tree.
* **Abort:** The node advertises an `/abort_grid_mapping` service (`std_srvs::SetBool`). Calling this service with `data: true` will trigger a `SIGABRT` signal, effectively stopping the node abruptly.

## ROS Interface

### Published Topics

* `/motor_direction` (`std_msgs::Int8`): Controls the direction of the LAC motor (1 for Zero/Up, 2 for Down). (Published by `LAC_Zero`, `LAC_Down`)
* `/linear_motor` (`std_msgs::Int32`): Triggers actions on a linear motor (1 or 2). (Published by `LAC_Zero`, `Slider_shift`)
* `/grid_number` (`std_msgs::Int32`): Publishes the current grid number being processed. (Published by `Electromagnet_On`)
* `/stroke_length` (`std_msgs::Int16`): Publishes calculated stroke lengths for the slider mechanism. (Published by `Slider_shift`)
* `/linear_servo` (`std_msgs::Int32`): (Advertised by `Slider_shift` but not used in the provided `tick()` code).

### Subscribed Topics

* `/rotation_time` (`std_msgs::Float32`): Provides dynamic sleep duration for `Dynamic_Sleep`.
* `/ui_stroke_length_publisher` (`std_msgs::Int16`): Provides stroke length configuration from a UI. (Used by `LAC_Zero`, `Electromagnet_On`, `LAC_Down`, `Slider_shift`)
* `/cycles_publisher` (`std_msgs::Int16`): Provides the number of cycles/grid size configuration from a UI. (Used by `LAC_Zero`, `Electromagnet_On`, `Slider_shift`)
* `/stroke_length_publisher` (`std_msgs::Int16`): Provides a base stroke length value. (Used by `Slider_shift`)
* `/multiplier_publisher` (`std_msgs::Int16`): Provides a multiplier value used in `Slider_shift`.
* `/set_joint_angle_publisher` (`std_msgs::Int32`): Provides data potentially used for timing after setting a joint angle. (Used by `Slider_shift`)

### Service Clients

* `/relay_toggle_channel` (`stm_interface::RelayControl`): Sends requests to toggle specific relay channels (3 for Pump/LAC_Down, 5 for Electromagnet). (Used by `Pump`, `Electromagnet_On`, `LAC_Down`, `Electromagnet_Off`)
* `/set_joint_angle` (`std_srvs::Trigger`): Calls a service to trigger a joint angle setting action. (Used by `Slider_shift`)

### Service Servers

* `pause_tree` (`std_srvs::Trigger`): Allows pausing and resuming the behavior tree execution.
* `/abort_grid_mapping` (`std_srvs::SetBool`): Allows forcefully stopping the node execution.

## Usage Examples

* **Pause the tree:**
    ```bash
    rosservice call /pause_tree "{}"
    ```
* **Resume the tree:**
    ```bash
    rosservice call /pause_tree "{}"
    ```
* **Abort the tree:**
    ```bash
    rosservice call /abort_grid_mapping "data: true"
    ```

## Configuration

* The Behavior Tree structure is currently hardcoded as an XML string within the `main` function. For more flexibility, consider loading it from an external `.xml` file using `factory.createTreeFromFile()`.
* Input values like cycle limits, stroke lengths, and multipliers are primarily controlled via subscribed topics (e.g., `/cycles_publisher`, `/ui_stroke_length_publisher`, `/multiplier_publisher`). Ensure nodes are publishing to these topics for correct operation.

## Contribution

Nilanjan Chowdhury

```
