# F1_Tenth_Race

ROS1 Melodic code for the F1TENTH autonomous racing platform, covering both the introductory simulation labs (RViz/Gazebo) and the code that runs on the physical car.

## Repository layout

```text
labs/               Simulation labs (RViz), progressing from ROS basics to a reactive driving stack
F1_Tenth_Car/        Real-car code, one self-contained catkin package per project checkpoint
```

### `labs/` — simulation labs

Each lab is a standalone catkin package or script set used with the simulated car in RViz/Gazebo.

- **lab0/beginner_tutorials** — ROS fundamentals: a custom `Num` message/service, `talker`/`listener` pub-sub nodes, an `add_two_ints` client/server, and a `commander` node that drives a turtlesim turtle via `Twist` messages.
- **lab1/src/mohammadnasr_roslab** — First LIDAR package: `lidar_processing_node.py` subscribes to `/scan` and publishes the closest/farthest point distances; `scan-subscriber.py` and `problem_C.py` are supporting exercises. Includes a custom `scan_range.msg` and recorded bagfiles.
- **Lab2** — `safetyNode.py`, an automatic-braking node that watches `/scan` and `/odom` and publishes `/brake` (`AckermannDriveStamped`) and `/brake_bool` when time-to-collision drops below a threshold. Builds on the lab1 package's LIDAR scripts.
- **Lab3** — `pid_control.py` / `pid_control.cpp`, a wall-following node that PID-controls steering off the left/right LIDAR wall distance (with an included `Lab 3.pdf` writeup).
- **Lab4** — `follow_gap.py` / `follow_gap.cpp`, a reactive "follow-the-gap" driving node that finds the widest free gap in the LIDAR scan and steers toward its midpoint.

Each lab folder only contains the files that are new or changed for that lab; earlier labs' unchanged scripts live in the package that introduced them (mainly `lab1/src/mohammadnasr_roslab`).

### `F1_Tenth_Car/` — real car code

Each `P*/car` directory is an independent catkin package (all named `car` in their own `package.xml`) checked out from the class-provided car template and modified for that checkpoint. They are meant to be built one at a time in their own workspace, not together.

- **P1** — Vision-based lane following and object detection: a semantic-segmentation LaneFollower (`src/control/control.cpp`, `src/semantic_segmentation/semantic_segmentation.cpp`, LibTorch models under `models/`), YOLO-based object detection/tracking (`scripts/car/object_detection/`), and a LIDAR-based braking/TTC safety node (`scripts/car/brake.py`). Includes launch files for camera, control, object detection, and semantic segmentation, plus recorded bagfiles.
- **P2** — PID wall-following on the real car (`scripts/car/pid.py`), the on-car counterpart to the `labs/Lab3` simulation node, tuned with different velocity constants.
- **P3** — Reactive follow-the-gap driving on the real car (`scripts/car/reactive_gap.py`), the on-car counterpart to `labs/Lab4`.

Where the real-car and simulation code implement the same algorithm (e.g. PID wall following, follow-the-gap), both versions are kept intentionally — the simulation version is developed/tested in RViz first, then adapted for the real car's hardware topics and constants.

## Requirements

- ROS1 Melodic
- Standard F1TENTH stack topics/messages: `sensor_msgs/LaserScan`, `nav_msgs/Odometry`, `ackermann_msgs/AckermannDriveStamped`
- P1 additionally requires OpenCV, `cv_bridge`, and LibTorch (C++) for the vision/segmentation pipeline
