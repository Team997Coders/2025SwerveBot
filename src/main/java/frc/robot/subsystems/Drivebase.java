// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import swervelib.SwerveModuleThingy;
import swervelib.SwerveModuleConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModuleLocations;
import frc.robot.Constants.DriveConstants.SwerveModules;

public class Drivebase extends SubsystemBase {
  private final double DRIVE_REDUCTION = 1.0 / 6.75;
  private final double NEO_FREE_SPEED = 5820.0 / 60.0;
  private final double WHEEL_DIAMETER = 0.1016;
  private final double MAX_VELOCITY = NEO_FREE_SPEED * DRIVE_REDUCTION * WHEEL_DIAMETER * Math.PI;
  private final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (ModuleLocations.dist / Math.sqrt(2.0));

  private final double MAX_VOLTAGE = 12;

  private AHRS gyro;

  private SwerveModuleThingy frontLeft = new SwerveModuleThingy(SwerveModules.frontLeft, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModuleThingy frontRight = new SwerveModuleThingy(SwerveModules.frontRight, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModuleThingy backLeft = new SwerveModuleThingy(SwerveModules.backLeft, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModuleThingy backRight = new SwerveModuleThingy(SwerveModules.backRight, MAX_VELOCITY, MAX_VOLTAGE);

  //                                                       0           1          2         3
  private SwerveModuleThingy[] modules = new SwerveModuleThingy[] { frontLeft, frontRight, backLeft, backRight };

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      ModuleLocations.frontLeft,
      ModuleLocations.frontRight,
      ModuleLocations.backLeft,
      ModuleLocations.backRight);

  private SwerveDrivePoseEstimator poseEstimator;

  private SwerveDriveOdometry odometry;

  private Field2d field = new Field2d();

  private SlewRateLimiter slewRateX = new SlewRateLimiter(DriveConstants.slewRate);
  private SlewRateLimiter slewRateY = new SlewRateLimiter(DriveConstants.slewRate);

  private static XboxController driveStick = new XboxController(0);

  private BooleanEntry fieldOrientedEntry;

  private Camera frontCamera;
  private Camera backCamera;

  /** Creates a new Drivebase. */
  public Drivebase(AHRS gyro, Camera frontCamera, Camera backCamera) {
    var inst = NetworkTableInstance.getDefault();
    var table = inst.getTable("SmartDashboard");
    this.fieldOrientedEntry = table.getBooleanTopic("Field Oriented").getEntry(true);
    this.gyro = gyro;

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), getPositions());

    poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), getPositions(), odometry.getPoseMeters());

    this.frontCamera = frontCamera;
    this.backCamera = backCamera;
    
    SmartDashboard.putData("Field", field);        
  }

  public double getFieldAngle() {
    return -gyro.getYaw();
  }

  public void fieldOrientedDrive(double speedX, double speedY, double rot) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rot,
        Rotation2d.fromDegrees(getFieldAngle()));
    this.drive(speeds);
  }

  public void robotOrientedDrive(double speedX, double speedY, double rot) {
    ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, rot);
    this.drive(speeds);
  }

  public void defaultDrive(double speedX, double speedY, double rot) {
    defaultDrive(speedX, speedY, rot, true);
  }

  public void defaultDrive(double speedX, double speedY, double rot, boolean slew) {
    if (slew) {
      speedX = slewRateX.calculate(speedX);
      speedY = slewRateY.calculate(speedY);
    }

    if (this.fieldOrientedEntry.get(true)) {
      fieldOrientedDrive(speedX, speedY, rot);
    } else {
      robotOrientedDrive(speedX, speedY, rot);
    }
  }

  /** drive:
   * Move the robot. Given the requested chassis speed (where do we want to go) in meters/sec and radians.
   * moduleStates are in meters/sec and radians (for rotation).
   * This can be a source of angle mismatch degrees <> radians
   */
  private void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds, new Translation2d(0, 0));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY);

    this.frontLeft.drive(moduleStates[0]);
    this.frontRight.drive(moduleStates[1]);
    this.backLeft.drive(moduleStates[2]);
    this.backRight.drive(moduleStates[3]);

    SmartDashboard.putNumber("BL Target Angle", moduleStates[2].angle.getDegrees());
  }

  public double getMaxVelocity() {
    return MAX_VELOCITY;
  }

  public double getMaxAngleVelocity() {
    return MAX_ANGULAR_VELOCITY;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose2d) {
    poseEstimator.resetPosition(gyro.getRotation2d(), getPositions(), pose2d);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  @Override
  public void periodic() {

    var positions = getPositions();

    odometry.update(gyro.getRotation2d(), positions);

    poseEstimator.update(gyro.getRotation2d(), positions);
    
    this.frontCamera.update(poseEstimator);
    this.backCamera.update(poseEstimator);
  }
}
