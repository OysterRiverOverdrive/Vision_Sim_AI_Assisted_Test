# MAXSwerve Vision & Pathfinding Simulation (FRC)

An AI-assisted, simulation-ready FRC swerve drivetrain codebase built on the **REV Robotics 3" MAXSwerve Java Template** (WPILib 2026)[cite: 2]. This project adds full physics simulation, simulated AprilTag vision with Kalman sensor fusion, pose-health diagnostics, and dynamic obstacle-avoidance pathfinding—allowing mentors and students to write, test, and verify competition code on a desktop before deploying to a physical robot.


Quick Start: Driving & Testing in the 3D Simulator
Anyone on the team can launch this project on a laptop without needing a physical robot. Start the WPILib Robot Simulator in Teleoperated mode, open AdvantageScope to the 3D Field view, and drag the robot's position topics (TrueSimPose as a solid robot, plus Drive/Pose and Vision/EstimatedPose as colored "ghost" robots) onto the field. We intentionally built simulated wheel slip and gyro drift into the physics so you can watch how the two ghosts separate from the real robot when driving through camera blind spots—and how the AprilTag camera automatically snaps the robot's estimated position back on track as soon as a tag comes back into view. From any spot on the field, you can test-drive manually with the joysticks, press Start to re-zero the robot's forward heading, press X to "kidnap" the robot (simulating a sudden tracking jump to (5.0, 5.0) so you can watch the camera recover), or hold the A button to watch the robot automatically plan a collision-free path around field obstacles straight to the Blue Human Player Station.

---

## Key Modifications from the Stock REV Template

### 1. Swerve Physics & AdvantageScope 3D Telemetry (`MAXSwerveModule.java`, `DriveSubsystem.java`)
* **Simulated Motor Physics (`SparkMaxSim`):** Added `RobotBase.isSimulation()` hooks using `SparkMaxSim` and `DCMotor` models (NEO drive / NEO 550 azimuth) so wheel velocities, positions, and steering angles update every 20 ms in desktop simulation without affecting real roboRIO hardware[cite: 2].
* **Simulated Gyro & Realistic Drift Modeling:** Implemented a simulated gyro yaw integrator (`getGyroRotation2d()`) alongside a ground-truth pose tracker (`SmartDashboard/Drive/TrueSimPose`) with configurable wheel-slip and gyro-drift biases to test real-world odometry error in simulation[cite: 2].
* **3D Field Visualization:** Added WPILib `StructPublisher` topics (`SmartDashboard/Drive/Pose`, `SmartDashboard/Drive/TrueSimPose`, and `SmartDashboard/Drive/ModuleStates`) for live 3D chassis and swerve-module rendering in **AdvantageScope**[cite: 2].
* **Controller Deadband Tuning (`Constants.java`):** Adjusted `OIConstants.kDriveDeadband` to eliminate resting thumbstick yaw drift[cite: 2].

### 2. Simulated AprilTag Vision & Sensor Fusion (`VisionSubsystem.java`, `DriveSubsystem.java`)
* **PhotonLib Virtual Camera (`VisionSystemSim`):** Configured a simulated camera (`FrontCamera`) with physical mounting offsets (`Transform3d`), 800x600 resolution, 70° FOV, and 25 ms latency rendering official competition `AprilTagFieldLayout` targets[cite: 2].
* **Kalman Filter Pose Fusion (`SwerveDrivePoseEstimator`):** Upgraded `DriveSubsystem` from pure wheel `SwerveDriveOdometry` to `SwerveDrivePoseEstimator`, blending latency-compensated AprilTag measurements into the robot's 50 Hz odometry[cite: 2].
* **Dynamic Trust Scaling & Filtering:** Added distance- and tag-count-weighted standard deviations (`Matrix<N3, N1>`), single-tag ambiguity rejection (`poseAmbiguity > 0.2`), and multi-tag yaw drift correction while keeping single-tag heading locked to the gyro[cite: 2].

### 3. Driver Station Pose-Health Diagnostics (`DriveSubsystem.java`)
* **Green / Yellow / Orange Status Indicators:** Publishes live divergence (`PoseHealth/GhostGapMeters`) and camera staleness (`PoseHealth/VisionAgeSec`) between the vision estimate and the fused robot pose (`PoseHealth/Green_TightLock`, `PoseHealth/Yellow_Drifting`) to warn drivers on Glass/Shuffleboard if odometry drifts during extended camera blind spots[cite: 2].

### 4. Closed-Loop Alignment & PathPlanner Obstacle Avoidance (`DriveToPoseCommand.java`, `DriveSubsystem.java`, `RobotContainer.java`)
* **Custom `DriveToPoseCommand`:** Built a 3-axis `ProfiledPIDController` command (X, Y, and continuous Theta) for smooth trapezoidal-profile docking to arbitrary field coordinates[cite: 2].
* **PathPlanner `AutoBuilder` Integration:** Added robot-relative kinematics helpers (`getRobotRelativeSpeeds()`, `driveRobotRelative()`) and configured holonomic `AutoBuilder` with alliance-flipping support for dynamic, collision-free pathfinding around field obstacles (`AutoBuilder.pathfindToPose`)[cite: 5].

---

## Development Effort & AI-Assisted Workflow Log
* **Estimated Active Development Time:** **~6 to 8 hours** across multiple short evening/mentor sessions (from initial WPILib/REV template setup through physics simulation, multi-tag Kalman fusion, custom PID alignment, PathPlanner obstacle avoidance, and Git workflow setup)[cite: 2, 5].
* **Development Approach:** Built incrementally using an AI-assisted pair-programming workflow (Google Gemini) to explain WPILib/REVLib architecture line-by-line, verify builds step-by-step, and establish a transferable mentor-to-student coding workflow[cite: 2].

---

## Before Deploying to the Physical Robot (`Constants.java` & `DriveSubsystem.java`)
All simulation physics and virtual cameras are isolated behind `RobotBase.isSimulation()` and automatically disable on a roboRIO[cite: 2]. Before deploying to the physical chassis:
1. **CAN IDs & Dimensions:** Update motor CAN IDs, wheel base/trackwidth, and module angular offsets in `Constants.java`[cite: 2].
2. **Gyro Hardware:** Swap the template's `ADIS16470_IMU` in `DriveSubsystem.java` for the physical **NavX (`AHRS`)** library[cite: 2].
3. **Camera Offsets & Names:** Match `kCameraName` and `kRobotToCam` (`Transform3d`) in `VisionSubsystem.java` to the physical cameras configured on the Orange Pi[cite: 2].