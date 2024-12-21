package frc.robot.commands;

import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Camera;

public class GoToTag extends Command {

  private final Drivebase drivebase;
  private final Camera frontCamera;
  private final Double radius;
  private Command currentPath;

  public GoToTag(Drivebase drivebase, Camera frontCamera, Double radius) 
  {
    this.drivebase = drivebase;
    this.frontCamera = frontCamera;
    this.radius = radius;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if (frontCamera.get_tag_Yaw() != 0)
    {
    
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    this.currentPath.schedule();
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
