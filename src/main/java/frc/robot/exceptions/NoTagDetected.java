package frc.robot.exceptions;

import frc.robot.subsystems.Drivebase;


public class NoTagDetected extends Exception
{   
    Drivebase drivebase;
    public NoTagDetected(String message)
    {
        super(message);
    }
}