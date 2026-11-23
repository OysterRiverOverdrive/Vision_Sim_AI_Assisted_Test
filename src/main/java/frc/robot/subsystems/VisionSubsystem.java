package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  // Must match the camera name configured in PhotonVision (http://localhost:5800)
  private static final String kCameraName = "FrontCamera";

  // Camera mounting offset from the center of the robot floor:
  // 10 inches forward (+X), centered left/right (0 Y), 12 inches up (+Z), tilted 15 degrees up
  private static final Transform3d kRobotToCam = new Transform3d(
      new Translation3d(Units.inchesToMeters(10.0), 0.0, Units.inchesToMeters(12.0)),
      new Rotation3d(0.0, Math.toRadians(-15.0), 0.0));

  // Baseline trust (standard deviations in [x meters, y meters, theta radians])
  private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, 999999.0);
  private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.08, 0.08, 5.0);

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> estimationStdDevs);
  }

  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_poseEstimator;
  private final VisionConsumer m_visionConsumer;
  private final Supplier<Pose2d> m_simPoseSupplier;

  // Simulation-only objects
  private VisionSystemSim m_visionSim;
  private PhotonCameraSim m_cameraSim;

  // Publishes the vision-calculated robot pose so AdvantageScope can display it
  private final StructPublisher<Pose2d> m_visionPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("SmartDashboard/Vision/EstimatedPose", Pose2d.struct).publish();

  public VisionSubsystem(VisionConsumer visionConsumer, Supplier<Pose2d> simPoseSupplier) {
    m_visionConsumer = visionConsumer;
    m_simPoseSupplier = simPoseSupplier;
    m_camera = new PhotonCamera(kCameraName);

    // Load the official competition AprilTag map
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    m_poseEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        kRobotToCam);

    if (RobotBase.isSimulation()) {
      m_visionSim = new VisionSystemSim("main");
      m_visionSim.addAprilTags(fieldLayout);

      SimCameraProperties cameraProps = new SimCameraProperties();
      cameraProps.setCalibration(800, 600, Rotation2d.fromDegrees(70.0));
      cameraProps.setFPS(30.0);
      cameraProps.setAvgLatencyMs(25.0);

      m_cameraSim = new PhotonCameraSim(m_camera, cameraProps);
      m_visionSim.addCamera(m_cameraSim, kRobotToCam);
    }
  }

  private Matrix<N3, N1> getEstimationStdDevs(
      EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
    int numTags = 0;
    double avgDist = 0.0;

    for (var tgt : targets) {
      var tagPose = m_poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) {
        continue;
      }
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation()
          .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
    }

    if (numTags == 0) {
      return kSingleTagStdDevs;
    }

    avgDist /= numTags;

    // Reject single-tag readings that are too far away
    if (numTags == 1 && avgDist > 4.0) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    // Start with multi-tag or single-tag baseline, then scale quadratically with distance
    Matrix<N3, N1> estStdDevs = (numTags > 1) ? kMultiTagStdDevs : kSingleTagStdDevs;
    return estStdDevs.times(1.0 + (avgDist * avgDist / 10.0));
  }

  @Override
  public void periodic() {
    for (var result : m_camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> visionEst = m_poseEstimator.update(result);
      visionEst.ifPresent(est -> {
        var targets = result.getTargets();
        // Reject single-tag frames with high corner ambiguity
        if (targets.size() == 1 && targets.get(0).getPoseAmbiguity() > 0.2) {
          return;
        }
        Pose2d estPose2d = est.estimatedPose.toPose2d();
        Matrix<N3, N1> stdDevs = getEstimationStdDevs(est, targets);
        m_visionPosePublisher.set(estPose2d);
        m_visionConsumer.accept(estPose2d, est.timestampSeconds, stdDevs);
      });
    }
  }

  @Override
  public void simulationPeriodic() {
    if (m_visionSim != null) {
      // Feed the simulated drivetrain's true position into the virtual camera world
      m_visionSim.update(m_simPoseSupplier.get());
    }
  }
}