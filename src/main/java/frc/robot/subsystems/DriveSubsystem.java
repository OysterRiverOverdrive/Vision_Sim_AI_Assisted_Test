// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Twist2d;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry; replaced by SwerveDrivePoseEstimator and twist2d
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Odometry class for tracking robot pose
// PoseEstimator fuses wheel/gyro odometry with AprilTag vision measurements
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getGyroRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d());

  private double m_simYawDeg = 0.0;
  private Pose2d m_simTruePose = new Pose2d();

  private final StructPublisher<Pose2d> m_posePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("SmartDashboard/Drive/Pose", Pose2d.struct).publish();

  private final StructPublisher<Pose2d> m_truePosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("SmartDashboard/Drive/TrueSimPose", Pose2d.struct).publish();

  private final StructArrayPublisher<SwerveModuleState> m_moduleStatePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("SmartDashboard/Drive/ModuleStates", SwerveModuleState.struct).publish();

  /** Creates a new DriveSubsystem. */
 /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // Load physical robot specs from PathPlanner GUI settings (or use MAXSwerve defaults)
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Fallback defaults for a standard ~50 kg MAXSwerve chassis so sim works immediately
      ModuleConfig moduleConfig = new ModuleConfig(
          0.0381, // 1.5 in wheel radius (3-inch MAXSwerve wheel)
          DriveConstants.kMaxSpeedMetersPerSecond,
          1.2,    // Wheel coefficient of friction
          DCMotor.getNEO(1).withReduction(4.71),
          40.0,   // Current limit (amps)
          1);
      config = new RobotConfig(
          50.0, // Robot mass (kg)
          6.0,  // Moment of inertia (kg*m^2)
          moduleConfig,
          new Translation2d(0.336, 0.336),   // Front Left
          new Translation2d(0.336, -0.336),  // Front Right
          new Translation2d(-0.336, 0.336),  // Rear Left
          new Translation2d(-0.336, -0.336)); // Rear Right
    }

    // Configure PathPlanner's AutoBuilder for holonomic (swerve) pathfinding
    AutoBuilder.configure(
        this::getPose,                // Pose supplier
        this::resetOdometry,          // Reset odometry method
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier (MUST BE ROBOT RELATIVE)
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Robot-relative drive consumer
        new PPHolonomicDriveController(
            new PIDConstants(3.0, 0.0, 0.0), // Translation (X/Y) PID
            new PIDConstants(4.0, 0.0, 0.0)  // Rotation (Theta) PID
        ),
        config,
        () -> {
          // Flip paths automatically if on the Red Alliance
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

@Override
public void periodic() {
    // Update the pose estimator with wheel encoders and gyro
    m_poseEstimator.update(
        getGyroRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    m_posePublisher.set(getPose());
    m_truePosePublisher.set(m_simTruePose);
    m_moduleStatePublisher.set(new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    });
  }


  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public Pose2d getTrueSimPose() {
    return m_simTruePose;
  }
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
public void resetOdometry(Pose2d pose) {
    m_simTruePose = pose;
    m_poseEstimator.resetPosition(
        getGyroRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Drives the robot using exact field-relative velocities (m/s and rad/s)
   * for closed-loop PID and Drive-to-Pose control.
   */
  public void driveFieldRelative(double xSpeedMetersPerSec, double ySpeedMetersPerSec, double rotRadPerSec) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeedMetersPerSec, ySpeedMetersPerSec, rotRadPerSec, getPose().getRotation()));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

/**
   * Returns the robot's current velocity relative to the chassis (forward, sideways, rotation).
   * Required by PathPlanner's AutoBuilder.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Drives the robot using robot-relative ChassisSpeeds (m/s and rad/s).
   * Required by PathPlanner's AutoBuilder.
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    m_simYawDeg = 0.0;
    // Keep X/Y position, reset heading to 0 deg across both estimator and sim truth
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getGyroRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

// helps determine deviation between vision and odometry.
  private Pose2d m_lastVisionPose = new Pose2d();
  private double m_lastVisionTimestamp = -10.0;

  /** Feeds a latency-compensated AprilTag pose into the Kalman filter. */
  
public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) 
  {
    m_poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    m_lastVisionPose = visionRobotPoseMeters;
    m_lastVisionTimestamp = Timer.getFPGATimestamp();
  }

  private Rotation2d getGyroRotation2d() {
    if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
      return Rotation2d.fromDegrees(m_simYawDeg);
    }
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ));
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what the gyro would report.
    double dt = 0.02;
    m_frontLeft.simulationPeriodic(dt);
    m_frontRight.simulationPeriodic(dt);
    m_rearLeft.simulationPeriodic(dt);
    m_rearRight.simulationPeriodic(dt);

    // 1. Advance the ground-truth simulated pose with zero drift
    ChassisSpeeds chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
    m_simTruePose = m_simTruePose.exp(
    // 0.85 factor is to simulate a 15% drivetrain loss in the simulation.    
    new Twist2d(
            chassisSpeeds.vxMetersPerSecond * 0.97 * dt,
            chassisSpeeds.vyMetersPerSecond * 0.97 * dt,
            chassisSpeeds.omegaRadiansPerSecond * dt));

    // 2. Advance the simulated gyro PLUS a small 2 deg/sec drift bias
    m_simYawDeg += Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond * dt) + 0.005;

    double ghostGapMeters = getPose().getTranslation()
    .getDistance(m_lastVisionPose.getTranslation());
double visionAgeSec = Timer.getFPGATimestamp() - m_lastVisionTimestamp;

boolean isGreen = (ghostGapMeters < 0.25) && (visionAgeSec < 0.5);
boolean isYellow = !isGreen && (ghostGapMeters < 1.0);
// Above 1.0 m gap (e.g., Green Ghost left far behind in midfield) -> both false ("Orange")

SmartDashboard.putNumber("PoseHealth/GhostGapMeters", ghostGapMeters);
SmartDashboard.putNumber("PoseHealth/VisionAgeSec", visionAgeSec);
SmartDashboard.putBoolean("PoseHealth/Green_TightLock", isGreen);
SmartDashboard.putBoolean("PoseHealth/Yellow_Drifting", isYellow);

  }
}
