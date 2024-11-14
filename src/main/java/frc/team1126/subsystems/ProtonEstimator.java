package frc.team1126.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// 
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ProtonEstimator {
    
PhotonPoseEstimator[] m_poseEstimators;

PhotonCamera m_front;
PhotonCamera m_back;
PhotonCamera m_left;
PhotonCamera m_right;

AprilTagFieldLayout fieldLayout;

public ProtonEstimator() {
    fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    m_front = new PhotonCamera("front");
    m_back = new PhotonCamera("back");
    m_left = new PhotonCamera("left");
    m_right = new PhotonCamera("right");
    
}

}
