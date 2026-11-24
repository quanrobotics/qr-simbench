# Third-party software

QR-SimBench orchestrates, but does not include or modify, these upstream projects (installed from
RoboStack / conda-forge into `./env`):

| Project | Role | Licence |
|---|---|---|
| ROS 2 Jazzy | middleware, launch, rclpy | Apache-2.0 |
| Gazebo Harmonic (gz-sim 8) | physics simulation | Apache-2.0 |
| Nav2 (navigation2, nav2_bringup, nav2_simple_commander, nav2_minimal_tb3_sim) | navigation stack under test, reference robot and sandbox world | Apache-2.0 / BSD |
| rosbag2 + rosbag2_storage_mcap | recording | Apache-2.0 |
| MCAP (`mcap`, `mcap-ros2-support`) | reading recordings | MIT |
| RoboStack | conda packaging of ROS | BSD-3-Clause |
| Typer, Pydantic, PyYAML, NumPy, Matplotlib, Jinja2, Pillow | dependencies | MIT / BSD / PSF / HPND |
| PyArrow | Parquet episode tables, LeRobot-style export | Apache-2.0 |
| FastAPI, Uvicorn | dependencies | MIT / BSD-3-Clause |

Files in this repository derived from upstream:

| File | Derived from | Licence |
|---|---|---|
| `simulation/launch/qr_fleet.launch.py` | nav2_bringup `cloned_multi_tb3_simulation_launch.py` | Apache-2.0 |
| `simulation/gui/*.gui.config` | gz-sim 8 default GUI config | Apache-2.0 |
| `simulation/config/qr_robot_bridge.yaml` | nav2_minimal_tb3_sim `turtlebot3_waffle_bridge.yaml` (`/clock` removed) | Apache-2.0 |
| `simulation/maps/tb3_sandbox.{pgm,yaml}` | nav2_bringup, unmodified | Apache-2.0 |
| `simulation/robots/gz_waffle_gt*.sdf.xacro` | nav2_minimal_tb3_sim TurtleBot3 Waffle SDF, Gazebo PosePublisher added | Apache-2.0 |
| `simulation/worlds/aws_small_warehouse*`, `simulation/maps/aws_small_warehouse.*` | [AWS RoboMaker small warehouse world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world), imported | MIT-0 |

The TurtleBot3 is the upstream Nav2 reference robot; QR does not manufacture it.
