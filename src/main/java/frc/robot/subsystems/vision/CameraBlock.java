package frc.robot.subsystems.vision;

import java.util.List;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class CameraBlock 
{
    List<Camera> cameraList;

    public CameraBlock(List<Camera> cameraList)
    {
        this.cameraList = cameraList;
    }

    public void update(SwerveDrivePoseEstimator poseEstimator)
    {
        for (Camera camera: this.cameraList)
        {
            camera.update(poseEstimator);
        }
    }
}
