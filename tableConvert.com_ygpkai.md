| Phase                                      | Item                       | File / Package    | Description                 | Status   | Tested |
|--------------------------------------------|----------------------------|-------------------|-----------------------------|----------|--------|
| Phase 0 — Repo & Workspace                 | Create ROS 2 workspace     | ~/jax_ws          | Standard Humble workspace   | Complete |        |
| Phase 0 — Repo & Workspace                 | Initialize git repo        | root              | Version control             | Complete |        |
| Phase 0 — Repo & Workspace                 | .gitignore                 | root              | Ignore build/install/log    | Complete |        |
| Phase 0 — Repo & Workspace                 | colcon build works         | all               | Clean build, no errors      | Complete |        |
| Phase 1 — Robot Description (URDF/Xacro)   | Create description package | jax_description   | URDF-only package           |          |        |
| Phase 1 — Robot Description (URDF/Xacro)   | Base link defined          | jax.urdf.xacro    | base_link or base_footprint |          |        |
| Phase 1 — Robot Description (URDF/Xacro)   | Visual meshes load         | meshes/*          | STL/DAE visible in RViz     |          |        |
| Phase 1 — Robot Description (URDF/Xacro)   | Collision geometry defined | URDF              | Simplified shapes           |          |        |
| Phase 1 — Robot Description (URDF/Xacro)   | Inertial blocks defined    | URDF              | Mass + inertia per link     |          |        |
| Phase 1 — Robot Description (URDF/Xacro)   | Leg macro created          | leg.urdf.xacro    | Parametric leg definition   |          |        |
| Phase 1 — Robot Description (URDF/Xacro)   | 4 legs instantiated        | jax.urdf.xacro    | fl / fr / rl / rr           |          |        |
| Phase 1 — Robot Description (URDF/Xacro)   | Joint axes verified        | URDF              | Correct rotation directions |          |        |
| Phase 1 — Robot Description (URDF/Xacro)   | Joint limits set           | URDF              | Safe limits                 |          |        |
| Phase 1 — Robot Description (URDF/Xacro)   | No TF loops                | TF tree           | Single root                 |          |        |
| Phase 2 — State Publishing & Visualization | robot_state_publisher      | launch            | Publishes TF                | Complete |        |
| Phase 2 — State Publishing & Visualization | joint_state_publisher      | launch            | Manual joint control        | Complete |        |
| Phase 2 — State Publishing & Visualization | RViz config saved          | rviz/*.rviz       | Consistent view             | Complete |        |
| Phase 2 — State Publishing & Visualization | Display launch file        | display.launch.py | One-command bringup         |          |        |
| Phase 2 — State Publishing & Visualization | TF tree verified           | tf2_tools         | view_frames OK              | Complete |        |
| Phase 3 — ros2_control (Simulation)        | ros2_control block         | URDF              | <ros2_control> tag          |          |        |
| Phase 3 — ros2_control (Simulation)        | Transmissions defined      | URDF              | Joint ↔ actuator mapping    |          |        |
| Phase 3 — ros2_control (Simulation)        | Fake/system hardware       | URDF              | Sim-only interface          |          |        |
| Phase 3 — ros2_control (Simulation)        | Controller config YAML     | controllers.yaml  | All joints listed           |          |        |
| Phase 3 — ros2_control (Simulation)        | joint_state_broadcaster    | controller        | Active                      |          |        |
| Phase 3 — ros2_control (Simulation)        | trajectory controller      | controller        | Leg control                 |          |        |
| Phase 3 — ros2_control (Simulation)        | Controllers launch         | launch            | Auto-spawn                  |          |        |
| Phase 4 — Gazebo Simulation                | Gazebo package             | jax_gazebo        | Sim-only                    |          |        |
| Phase 4 — Gazebo Simulation                | Gazebo launch file         | gazebo.launch.py  | World + robot               |          |        |
| Phase 4 — Gazebo Simulation                | Gazebo plugins added       | URDF              | ros2_control + sensors      |          |        |
| Phase 4 — Gazebo Simulation                | Ground contact works       | sim               | No falling through floor    |          |        |
| Phase 4 — Gazebo Simulation                | Robot stands up            | sim               | Zero pose stable            |          |        |
| Phase 4 — Gazebo Simulation                | Joint control verified     | sim               | Commands move legs          |          |        |
| Phase 5 — Locomotion (Minimal)             | Gait node package          | jax_gait          | Custom logic                |          |        |
| Phase 5 — Locomotion (Minimal)             | Joint trajectory publisher | node              | Sends leg motions           |          |        |
| Phase 5 — Locomotion (Minimal)             | Stand pose defined         | YAML / code       | Neutral stance              |          |        |
| Phase 5 — Locomotion (Minimal)             | Walk gait defined          | code              | Simple crawl/trot           |          |        |
| Phase 5 — Locomotion (Minimal)             | Sim walk verified          | Gazebo            | Stable steps                |          |        |
| Phase 6 — Sensors & State                  | Battery message source     | battery_node      | Publishes BatteryState      |          |        |
| Phase 6 — Sensors & State                  | IMU integration            | imu_node          | Orientation data            |          |        |
| Phase 6 — Sensors & State                  | Sensor TF frames           | URDF              | imu_link etc                |          |        |
| Phase 6 — Sensors & State                  | State aggregation          | robot_state       | Mode / error flags          |          |        |
| Phase 7 — LCD Display (Pi)                 | Waveshare driver tested    | standalone        | SPI works                   |          |        |
| Phase 7 — LCD Display (Pi)                 | LCD ROS package            | jax_lcd           | Python node                 |          |        |
| Phase 7 — LCD Display (Pi)                 | Battery subscribe          | lcd_node          | Shows %                     |          |        |
| Phase 7 — LCD Display (Pi)                 | Mode display               | lcd_node          | IDLE/WALK                   |          |        |
| Phase 7 — LCD Display (Pi)                 | Refresh throttling         | lcd_node          | Low CPU                     |          |        |
| Phase 8 — Hardware Interface (Real Robot)  | Servo hardware interface   | ros2_control      | Custom plugin               |          |        |
| Phase 8 — Hardware Interface (Real Robot)  | Replace fake hardware      | URDF              | Real IO                     |          |        |
| Phase 8 — Hardware Interface (Real Robot)  | Servo calibration          | config            | Zero offsets                |          |        |
| Phase 8 — Hardware Interface (Real Robot)  | Joint limits enforced      | hardware          | Safety                      |          |        |
| Phase 8 — Hardware Interface (Real Robot)  | Emergency stop logic       | node              | Kill motion                 |          |        |
| Phase 9 — Bringup & Deployment             | Bringup launch file        | bringup.launch.py | Full system                 |          |        |
| Phase 9 — Bringup & Deployment             | Headless startup           | systemd           | Auto-launch                 |          |        |
| Phase 9 — Bringup & Deployment             | Logging configured         | ros2              | Diagnostics                 |          |        |
| Phase 9 — Bringup & Deployment             | Pi performance tuned       | OS                | CPU/GPU                     |          |        |
| Phase 9 — Bringup & Deployment             | Final walk test            | real robot        | Success                     |          |        |
| Phase 10 — Nice-to-Have (Optional)         | Parameter tuning           | YAML              | Gait speed                  |          |        |
| Phase 10 — Nice-to-Have (Optional)         | Teleop input               | joystick          | Manual control              |          |        |
| Phase 10 — Nice-to-Have (Optional)         | Diagnostics topic          | /diagnostics      | Health                      |          |        |
| Phase 10 — Nice-to-Have (Optional)         | OTA update path            | git               | Field updates               |          |        |
| Final Milestone                            | Robot stands               | No drift          |                             |          |        |
| Final Milestone                            | Robot walks                | Controlled gait   |                             |          |        |
| Final Milestone                            | Status visible             | LCD working       |                             |          |        |
| Final Milestone                            | One-command startup        | Done              |                             |          |        |
