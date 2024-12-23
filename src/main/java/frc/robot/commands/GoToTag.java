package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.exceptions.NoTagDetected;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivebase;

public class goToTag extends Command {
  private final Drivebase drivebase;
  private final Camera frontCamera;

  /** Creates a new TimeDrive. */
  public goToTag(Drivebase drivebase, Camera frontCamera) {
    this.drivebase = drivebase;
    this.frontCamera = frontCamera;

    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(new Pose2d(),new Pose2d());
    try
    {
    waypoints = PathPlannerPath.waypointsFromPoses(
        drivebase.getPose(),
        frontCamera.getTagPose()
    );
    } catch(NoTagDetected e)
    {
        this.cancel();
    }
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );
    path.preventFlipping = true;

    AutoBuilder.followPath(path);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}