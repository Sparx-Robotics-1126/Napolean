package frc.team1126.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team1126.Constants.VisionConstants;

import java.util.Optional;

// 
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ProtonEstimator {
    
PhotonPoseEstimator[] m_poseEstimators;

private static PhotonCamera m_front;
private static PhotonCamera m_back;
private static PhotonCamera m_left;
private static PhotonCamera m_right;

private final PIDController m_yVisionPidController = new PIDController(0.033, 0.0, 0.005);
private final PIDController m_xVisionPidController = new PIDController(0.033, 0.0, 0.005);

AprilTagFieldLayout fieldLayout;

public ProtonEstimator() {
    fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    m_front = new PhotonCamera("front");
    m_back = new PhotonCamera("back");
    m_left = new PhotonCamera("left");
    m_right = new PhotonCamera("right");

    m_yVisionPidController.setTolerance(0.5);
    m_xVisionPidController.setTolerance(0.5);
    
    m_poseEstimators = new PhotonPoseEstimator[] {
        new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.kFrontCameraLocation)
            //may need to add other cameras after this
    };

}
 public void updatePoseEstimationWithFilter() {
            //Pose2d currentPose = m_poseEstimators.getEstimatedPosition();
            for (PhotonPoseEstimator poseEstimator : m_poseEstimators) {
                // print out the time for this line to run 
                Optional<EstimatedRobotPose> pose = poseEstimator.update();
                if (pose.isPresent()) {
                    Pose3d pose3d = pose.get().estimatedPose;
                    Pose2d pose2d = pose3d.toPose2d();
                    if (
                        pose3d.getX() >= -VisionConstants.VISION_FIELD_MARGIN &&
                        pose3d.getX() <= VisionConstants.FIELD_LENGTH + VisionConstants.VISION_FIELD_MARGIN &&
                        pose3d.getY() >= -VisionConstants.VISION_FIELD_MARGIN &&
                        pose3d.getY() <= VisionConstants.FIELD_WIDTH + VisionConstants.VISION_FIELD_MARGIN &&
                        pose3d.getZ() >= -VisionConstants.VISION_Z_MARGIN &&
                        pose3d.getZ() <= VisionConstants.VISION_Z_MARGIN
                    ) {
                        double sum = 0.0;
                        for (PhotonTrackedTarget target : pose.get().targetsUsed) {
                            Optional<Pose3d> tagPose =
                                fieldLayout.getTagPose(target.getFiducialId());
                            if (tagPose.isEmpty()) continue;
                            sum += currentPose.getTranslation().getDistance(tagPose.get().getTranslation().toTranslation2d());
                        }

                        int tagCount = pose.get().targetsUsed.size();
                        double stdScale = Math.pow(sum / tagCount, 2.0) / tagCount;
                        double xyStd = VisionConstants.VISION_STD_XY_SCALE * stdScale;
                        double rotStd = VisionConstants.VISION_STD_ROT_SCALE * stdScale;
                        //time this as well
                        m_poseEstimator.addVisionMeasurement(pose2d, pose.get().timestampSeconds, VecBuilder.fill(xyStd, xyStd, rotStd));
                        continue;
                    }
                }
            }
    }
//pipeline index - one of them is for red team, the other is for blue
public static void configureCameras(int pipelineIndex) {
    m_front.setPipelineIndex(pipelineIndex);
    m_back.setPipelineIndex(pipelineIndex);
    m_left.setPipelineIndex(pipelineIndex);
    m_right.setPipelineIndex(pipelineIndex);


}

}