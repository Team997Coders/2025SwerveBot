package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class Camera
{
    private PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    private PhotonPoseEstimator photonPoseEstimator;

    private List<PhotonPipelineResult> results;

    public Camera(String cameraName, Transform3d robotToCamera)
    {
        this.camera = new PhotonCamera(cameraName);

        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

        this.results = null;
    }

    public void update(SwerveDrivePoseEstimator poseEstimator)
    {
        this.results = this.camera.getAllUnreadResults();
        if (!this.results.isEmpty())
        {
            for (PhotonPipelineResult result: results)
            {
                Optional<EstimatedRobotPose> estimatedRobotPose = this.photonPoseEstimator.update(result);
                if (estimatedRobotPose.isPresent())
                {
                    poseEstimator.addVisionMeasurement(estimatedRobotPose.orElseThrow().estimatedPose.toPose2d(), result.getTimestampSeconds());
                }
            }
            
        }
    }
}