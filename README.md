# nxp-uwb-ros2

`nxp-uwb-ros2` is a ROS2 driver for the NXP SR150 Ultra-Wideband (UWB) chip, which implements precise ranging capabilities. This package is designed for applications requiring accurate distance measurement and positioning between devices using UWB technology.

## Features

- Support for two-way ranging (TWR) and time difference of arrival (TDoA).
- Configurable parameters including roles, MAC addresses, and session settings.
- Publishes UWB ranging data as ROS2 messages.

## Requirements

- ROS2 Humble or later
- CMake 3.8 or later
- Required dependencies:
  - `rclcpp`
  - `std_msgs`
  - [`uwb_msgs`](https://github.com/rudislabs/uwb_msgs)
- Linux SR1XX Kernel driver [`NXP_SPI_Driver` V7](https://github.com/NXP/uwb-driver-testapp/tree/59bd749fff2d4a9a6de103bbe8f5e5565208b164)

## Linux kernel driver setup

The Linux SR1XX driver is an out-of-tree kernel driver, you need to patch the driver into your kernel.

For testing the [NXP NavQPlus](https://www.nxp.com/design/design-center/development-boards-and-designs/8MPNAVQ) is used in combination with a [Murata Type 2BP Evaluation kit](https://www.murata.com/products/connectivitymodule/ultra-wide-band/nxp/type2bp)

The NXPHovergames `meta-nxp-mr` Yocto image for the NXP NavQPlus already includes the patches to support the SR1XX kernel driver. But here are samples on how to patch your own kernel.
 - [0004-nxp-sr1xx-uwb-driver.patch](https://github.com/NXPHoverGames/meta-nxp-mr/blob/lf-6.6.23-2.0.0-scarthgap/recipes-kernel/linux/linux-imx/0004-nxp-sr1xx-uwb-driver.patch)
 - [0005-imx8mpnavq-enable-sr1xx-spi-driver-in-dts.patch](https://github.com/NXPHoverGames/meta-nxp-mr/blob/lf-6.6.23-2.0.0-scarthgap/recipes-kernel/linux/linux-imx/0005-imx8mpnavq-enable-sr1xx-spi-driver-in-dts.patch)

### NavQPlus & Murata Type 2BP Evaluation kit hardware setup

To connect the Murata Type 2BP Evaluation kit easily to the NavQPlus there is a reference design opensource PCB adapter board [schematic](./pcb/Schematic_XMR-NAVQ-TYPE2BP_2025-01-03.pdf) and [fabrication files](./pcb/XMR_NAVQ_TYPE2BP_FABRICATION_FILES.zip) published. You will need to manufacture these yourself or reference the schematics to make your own manual inter-connection.

## Installation

Clone this repository into your ROS2 workspace:

```bash
git clone https://github.com/NXPHoverGames/nxp-uwb-ros2 src/nxp-uwb-ros2
```

Install dependencies using `rosdep`:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
colcon build
```

## Usage

### Running the Node

The primary executable, `node`, initializes and starts UWB ranging based on the specified parameters.

```bash
ros2 run nxp-uwb-ros2 node
```

### Configuration

The node provides several parameters to configure the behavior of the driver. These can be set via a YAML file or command line arguments.

#### Parameters:

| Parameter             | Description                                   | Default Value       |
|-----------------------|-----------------------------------------------|---------------------|
| `role`                | Role of the UWB device                       | `twr_initiator`      |
| `device_mac_address`  | MAC address of the device                    | `[0x1111]`          |
| `dst_mac_address`     | Destination MAC addresses                    | `[0x2222]`          |
| `mac_is_extended`     | Whether MAC address is extended              | `false`             |
| `channel_id`          | UWB channel ID                               | `0x9`               |
| `session_id`          | UWB session ID                               | `1`                 |
| `ranging_interval`    | Ranging interval in milliseconds             | `50`                |
| `position`            | Device position (used for anchors)           | `[]`                |

Example YAML configuration file for TDoA controller:

```yaml
role: "tdoa_dl_anchor_controller"
device_mac_address: [0x1111]
dst_mac_address: [0x2222, 0x3333]
mac_is_extended: false
channel_id: 9
session_id: 1
ranging_interval: 50
position: [0.0, 0.0, 0.0]
```

Launch the node with the configuration file:

```bash
ros2 run nxp-uwb-ros2 node --ros-args --params-file config.yaml
```

### Roles

| Role                          | Description                                                | ROS2 Example Launch File   |
|-------------------------------|------------------------------------------------------------|----------------------------|
| twr_initiator                 | Initiates Two-Way Ranging (TWR) measurements.              | twr_initiator_launch.py    |
| twr_responder                 | Responds to Two-Way Ranging (TWR) measurements.            | twr_responder_launch.py    |
| tdoa_dl_tag                   | Tag used in Downlink Time Difference of Arrival (TDoA DL). | tdoa_tag_launch.py         |
| tdoa_dl_anchor_controller     | Controller anchor in TDoA DL.                              | tdoa_controller_launch.py  |
| tdoa_dl_anchor_controlee      | Controlee anchor in TDoA DL.                               | tdoa_controlee_x_launch.py |



### Topics

The node publishes the following topics:

| Topic Name                     | Message Type                            | Description                                |
|--------------------------------|-----------------------------------------|--------------------------------------------|
| `/sr1xx/<role>`                       | `uwb_msgs/UltraWideBandRanging`     | UWB ranging measurements and metadata      |

### Scripts

The scripts folder contains useful script processing `uwb_msgs/UltraWideBandRanging` data

See [Script README.md](./scripts/README.md) for more

| Script                       | Short Description                                                                                  |
|------------------------------|----------------------------------------------------------------------------------------------------|
| twr_point_publisher.py       | Converts Two-way ranging `uwb_msgs/UltraWideBandRanging` data to ROS2 `geometry_msgs/PointStamped` |
| ros_gz_twr_data_generator.py | Generates Two-way ranging `uwb_msgs/UltraWideBandRanging` data from Gazebo simulator               |
| gz_twr_point.py | Takes GZ position data and revereses it into UWB TWR data |
| demo_control.py | Allows for controlling the B3RB by having it go through a list of defined points including navigating to a dynamic UWB point |

## Development

### Code Structure

- **`src/sr1xx_node.cpp`**: Main implementation of the ROS2 node.
- **`src/sr1xx_driver.c`**: SR1XX driver implemenation.
- **`src/hal/`**: SR1XX Hardware abstraction layer and protocol implementations.
- **`standalone`**: [SR1XX Standalone application](./standalone/README.md) for testing hardware functionality.
- **`launch`**: ROS2 Launch files examples to start the SR1XX in various ranging modes

### Building

Ensure the workspace is built using `colcon build`. Add additional source files to `CMakeLists.txt` as needed.

## License

This project is licensed under the Apache License, Version 2.0. See the LICENSE file for details.

## Support

For questions or issues, please open an issue on the [GitHub repository](https://github.com/NXPHoverGames/nxp-uwb-ros2).
