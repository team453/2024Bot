package frc.robot.subsystems;


import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.helpers.LimelightHelpers;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final DriveSubsystem drivetrainSubsystem;
  
  private final AprilTagFieldLayout aprilTagFieldLayout;
  
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

private static final Vector<N3> stateStdDevs = VecBuilder.fill(
    VisionConstants.STATE_STD_DEV_X, 
    VisionConstants.STATE_STD_DEV_Y, 
    VisionConstants.STATE_STD_DEV_THETA
);

private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(
    VisionConstants.VISION_MEASUREMENT_STD_DEV_X, 
    VisionConstants.VISION_MEASUREMENT_STD_DEV_Y, 
    VisionConstants.VISION_MEASUREMENT_STD_DEV_THETA
);

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;

  public PoseEstimatorSubsystem(DriveSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    AprilTagFieldLayout layout;
    try {
        layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        var alliance = DriverStation.getAlliance();
    
        // Expanded if-else statement
        if (alliance.get() == DriverStation.Alliance.Blue) {
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } else {
            layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        }
    
    } catch(IOException e) {
        DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        layout = null;
        
    }
    this.aprilTagFieldLayout = layout;
    
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator =  new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
    
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  @Override
  public void periodic() {
    // Update pose estimator with the best visible target
LimelightHelpers.LimelightResults limelightResults = LimelightHelpers.getLatestResults("limelight");
double resultTimestamp = limelightResults.targetingResults.timestamp_LIMELIGHT_publish;

if (resultTimestamp != previousPipelineTimestamp && limelightResults.targetingResults.valid) {
    previousPipelineTimestamp = resultTimestamp;

    for (LimelightHelpers.LimelightTarget_Fiducial target : limelightResults.targetingResults.targets_Fiducials) {
        double fiducialId = target.fiducialID;
        
        // Get the tag pose from field layout - consider that the layout will be null if it failed to load
        Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose((int) fiducialId);
        
        if (target.ta <= VisionConstants.MaxPoseAmbiguity && fiducialId >= 0 && tagPose.isPresent()) {
            var targetPose = tagPose.get();
            Pose3d pose3d = target.getTargetPose_CameraSpace(); // Get camera to target pose
            // Convert Pose3d to Transform3d
            Transform3d camToTarget = new Transform3d(
            pose3d.getTranslation(),
            pose3d.getRotation()
            );
            
            Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

               var visionMeasurement = camPose.transformBy(VisionConstants.CAMERA_TO_ROBOT);
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
        }
    }
}

// Update pose estimator with drivetrain sensors
poseEstimator.update(
  drivetrainSubsystem.getGyroscopeRotation(),
  drivetrainSubsystem.getModulePositions());

field2d.setRobotPose(getCurrentPose());

    /* 
    // Update pose estimator with the best visible target
    var pipelineResult = photonCamera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
      if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
        var targetPose = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
      }
    }
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
      drivetrainSubsystem.getGyroscopeRotation(),
      drivetrainSubsystem.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
    */
    /*
        poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d("limelight"),Timer.getFPGATimestamp());
      
    
        // Update pose estimator with drivetrain sensors
         poseEstimator.update(
             drivetrainSubsystem.getGyroscopeRotation(),
             drivetrainSubsystem.getModulePositions());

         field2d.setRobotPose(getCurrentPose());
     */
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      drivetrainSubsystem.getGyroscopeRotation(),
      drivetrainSubsystem.getModulePositions(),
      newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

}