# Intent Prediction

Human intent prediction for a TurtleBot3 navigating among people (ROS 2 Humble, Ignition Gazebo, Nav2).

The robot detects walking humans, predicts whether each one will cross or walk into its path, and makes Nav2 slow down 
and steer around where the person **will be**. A lidar safety layer sits on top as a last line of defence.

## How it works

```
 camera ──┐
          ├──► perception ──► /human_tracks ──► prediction
 lidar ───┘    human_tracker                    intent_nav_bridge
                                                │
                                                ├──► /human_predicted_grid ──► Nav2 local costmap
                                                ├──► /speed_limit ───────────► Nav2 controller
                                                ├──► /human_intents
                                                └──► /intent_markers ────────► RViz

 Nav2 ──► /cmd_vel ──► safety ──► /cmd_vel_safe ──► robot
                       safety_zones
                          ▲
                          │
                        /scan
```

Nav2 does the driving to the goal you click in RViz. `prediction` never publishes `/cmd_vel`; it only shapes the costmap and the speed limit.

## Requirements

- Ubuntu 22.04, ROS 2 Humble
- Ignition Gazebo (`ros_gz_sim`, `ros_gz_bridge`), `turtlebot3_gazebo`, `turtlebot3_description`
- Nav2 (`nav2_bringup`), `slam_toolbox`
- Python: `ultralytics`, `numpy`, `opencv-python`

## Build

```bash
mkdir -p ~/intent_ws/src && cd ~/intent_ws/src
git clone https://github.com/girija-connect/intent-prediction.git .
cd ~/intent_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install ultralytics "numpy<2"   # not installed by rosdep; numpy<2 matches Humble's cv_bridge
colcon build --symlink-install
source install/setup.bash
```

The YOLO weights (`yolov8n.pt`, about 6 MB) download automatically on first run, so internet is needed once. 
For offline use, copy the file over and pass it in:

```bash
ros2 run perception human_tracker --ros-args -p model:=/path/to/yolov8n.pt
```

## Run

Everything (sim, Nav2, perception, prediction, safety, RViz):

```bash
ros2 launch bringup bringup.launch.py
```

Or in separate terminals:

```bash
ros2 launch sim sim.launch.py
ros2 launch navigation navigation.launch.py
ros2 launch bringup rviz.launch.py
```

In RViz, set a **2D Pose Estimate**, then send a **Nav2 Goal**. The office world has three walking people: 
one head-on (`colleague_headon`), one crossing (`worker_crossing`) and one slow walker (`person_slow`). 
Watch the intent labels and predicted paths in RViz, the speed limit dropping as they approach, and the safety zones lighting up when someone gets close.


### Re-mapping the office

```bash
ros2 launch sim sim.launch.py
ros2 launch mapping mapping.launch.py      # sim_mode:=false on the real robot
ros2 run nav2_map_server map_saver_cli -f src/mapping/maps/office
```


### 1. Perception (`human_tracker`)

1. YOLOv8 + ByteTrack detect people and keep a stable ID per person.
2. The foot pixel of each box is projected onto the ground plane to get a position.
3. The lidar range replaces that estimate when a matching return exists (more accurate).
4. A per-person Kalman filter outputs position, speed and heading in the `odom` frame.
5. A track is trusted after 6 detections and dropped after 2 s unseen.

### 2. Intent prediction (`intent_nav_bridge`)

Runs at 5 Hz. For each trusted human, the closest point of approach (CPA) between the human's and the robot's constant-velocity paths gives one label:

| Label | Meaning |
|---|---|
| `STANDING` | Human speed below 0.2 m/s |
| `MOVING_AWAY` | Paths are already diverging |
| `PASSING_BY` | Closest approach stays beyond 1.0 m |
| `CROSSING_PATH` | Closest approach within 1.0 m, not head-on |
| `APPROACHING_HEAD_ON` | Closest approach within 1.0 m, walking toward the robot |

What the labels do:

- **Costmap:** for `CROSSING_PATH` and `APPROACHING_HEAD_ON` with a CPA time of 5 s or less, the predicted path (growing discs over the next 2 s) is painted into `/human_predicted_grid`. Nav2's local costmap reads it through `human_layer`, so the planner avoids where the person will be. Other labels are already handled by the lidar obstacle layer.
- **Speed limit** (`/speed_limit`, as a percentage of top speed):
  - Head-on: full speed at 5 s to CPA, falling to the 30% floor at 1 s.
  - Crossing: 100% at 5 s, 50% once they cross within 2 s.
  - Anyone within 1 m: eased off regardless of label.
  - Never below 30%. Slows down immediately, recovers gradually (10% per tick).

### 3. Safety zones (`safety_zones`)

Circles around the robot, checked against `/scan` on every velocity command:

| Zone | Default radius | Action |
|---|---|---|
| Yellow | 0.75 m | Speed scaled to 50% |
| Red | 0.35 m | Full stop |

While driving, only obstacles in the direction of travel count (so a wall beside you does not freeze the robot). While rotating or standing, all directions count. No lidar data for 0.5 s means stop.

## Notes

- `prediction` also contains `intent_controller`, an old stand-alone driver. Never run it together with Nav2.
- YOLO weights (`*.pt`) are git-ignored.

## License

MIT, see [LICENSE](LICENSE).
