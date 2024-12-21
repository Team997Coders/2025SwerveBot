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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class Camera
{
    private PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPoseEstimator photonPoseEstimator;
    private static final Boolean updatePosition = true;

    public Camera(String cameraName, Transform3d robotToCamera)
    {
        this.camera = new PhotonCamera(cameraName);

        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    }

    public void update(SwerveDrivePoseEstimator poseEstimator)
    {
        if (updatePosition)
        {
            var results = this.camera.getLatestResult();
            if (results.hasTargets())
            {
                Optional<EstimatedRobotPose> estimatedRobotPose = this.photonPoseEstimator.update(results);
                if (estimatedRobotPose.isPresent())
                {
                    poseEstimator.addVisionMeasurement(estimatedRobotPose.orElseThrow().estimatedPose.toPose2d(), results.getTimestampSeconds());
                }
            }
        }
    }

    public Transform3d get_tag_Transform3d()
    {
        var results = this.camera.getLatestResult();
        if (results.hasTargets())
        {
            return results.getBestTarget().getBestCameraToTarget();
        }
        return new Transform3d();
    }

    public Pose2d get_tag_pose2d()
    {
        var results = this.camera.getLatestResult();
        if (results.hasTargets())
        {
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(results.getBestTarget().getFiducialId());
            if(tagPose.isPresent())
            {
                return tagPose.orElseThrow().toPose2d();
            }
        }
        return new Pose2d();
    }

    public double get_tag_Yaw()
    {
        var results = this.camera.getLatestResult();
        if (results.hasTargets())
        {
            return results.getBestTarget().getYaw();
        }
        return 0;
    }

    public double get_tag_Pitch()
    {
        var results = this.camera.getLatestResult();
        if (results.hasTargets())
        {
            return results.getBestTarget().getPitch();
        }
        return 0;
    }

    public Pose3d get_field_relative_pose()
    {
        var results = this.camera.getLatestResult();
        if (results.hasTargets())
        {
            PhotonTrackedTarget target = results.getBestTarget();
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()){
            return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), tagPose.orElseThrow(), new Transform3d());
            }
        }
        return new Pose3d();
    }

    public Translation2d robot_to_tag(Drivebase drivebase)
    {
        var results = this.camera.getLatestResult();
        if (results.hasTargets())
        {
            PhotonTrackedTarget target = results.getBestTarget();
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent())
            {
            return PhotonUtils.estimateCameraToTargetTranslation(PhotonUtils.getDistanceToPose(drivebase.getPose(), tagPose.orElseThrow().toPose2d()), Rotation2d.fromDegrees(-target.getYaw()));
            }
        }
        return new Translation2d();
    }

    public double get_distance_to_tag()
    {
        var results = this.camera.getLatestResult();
        if (results.hasTargets())
        {
            PhotonTrackedTarget target = results.getBestTarget();
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent())
            {
                return PhotonUtils.calculateDistanceToTargetMeters(this.photonPoseEstimator.getRobotToCameraTransform().getZ(), 
                tagPose.orElseThrow().getZ(), 
                this.photonPoseEstimator.getRobotToCameraTransform().getRotation().getY(), 
                Units.degreesToRadians(get_tag_Pitch()));
            }
        }
        return 0;
    }
}
