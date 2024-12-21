package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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
import frc.robot.exceptions.NoTagDetected;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class Camera
{
    private PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    private PhotonPoseEstimator photonPoseEstimator;

    public Camera(String cameraName, Transform3d robotToCamera)
    {
        this.camera = new PhotonCamera(cameraName);

        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    }

    public void update(SwerveDrivePoseEstimator poseEstimator)
    {
        var results = this.camera.getAllUnreadResults();
        if (!results.isEmpty())
        {
            for (int i = 0; i < results.size(); i++)
            {
                Optional<EstimatedRobotPose> estimatedRobotPose = this.photonPoseEstimator.update(results.get(i));
                if (estimatedRobotPose.isPresent())
                {
                    poseEstimator.addVisionMeasurement(estimatedRobotPose.orElseThrow().estimatedPose.toPose2d(), results.get(i).getTimestampSeconds());
                }
            }   
        }
    }

    public Pose2d getTagPose() throws NoTagDetected
    {
        var results = this.camera.getAllUnreadResults();
        if (!results.isEmpty())
        {
            return aprilTagFieldLayout.getTagPose(results.get(0).getBestTarget().getFiducialId()).get().toPose2d();
        } else
        {
            throw new NoTagDetected("No Tag Detected");
        }
    }
}

