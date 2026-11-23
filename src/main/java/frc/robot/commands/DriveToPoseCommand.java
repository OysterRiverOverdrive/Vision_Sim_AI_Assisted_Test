package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPoseCommand extends Command {
  private final DriveSubsystem m_drive;
  private final Pose2d m_targetPose;

  // Motion profile constraints (Max velocity 2.5 m/s, Max accel 2.5 m/s^2)
  private final ProfiledPIDController m_xController = new ProfiledPIDController(
      2.5, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 2.5));
  private final ProfiledPIDController m_yController = new ProfiledPIDController(
      2.5, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 2.5));
  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(
      4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI * 1.5, Math.PI * 2.0));

  // Publishes the target coordinate so AdvantageScope can display a target ghost
  private static final StructPublisher<Pose2d> m_targetPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("SmartDashboard/Drive/TargetPose", Pose2d.struct).publish();

  public DriveToPoseCommand(DriveSubsystem drive, Pose2d targetPose) {
    m_drive = drive;
    m_targetPose = targetPose;

    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_xController.setTolerance(0.03); // 3 cm
    m_yController.setTolerance(0.03); // 3 cm
    m_thetaController.setTolerance(Math.toRadians(2.0)); // 2 degrees

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = m_drive.getPose();
    m_xController.reset(currentPose.getX());
    m_yController.reset(currentPose.getY());
    m_thetaController.reset(currentPose.getRotation().getRadians());
    m_targetPublisher.set(m_targetPose);
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_drive.getPose();

    double xSpeed = m_xController.calculate(currentPose.getX(), m_targetPose.getX());
    double ySpeed = m_yController.calculate(currentPose.getY(), m_targetPose.getY());
    double rotSpeed = m_thetaController.calculate(
        currentPose.getRotation().getRadians(),
        m_targetPose.getRotation().getRadians());

    m_drive.driveFieldRelative(xSpeed, ySpeed, rotSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.driveFieldRelative(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return m_xController.atGoal() && m_yController.atGoal() && m_thetaController.atGoal();
  }
}