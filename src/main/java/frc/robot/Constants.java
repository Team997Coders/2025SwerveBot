// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import swervelib.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final double deadband = 0.08;
    public static final int currentLimit = 40;
    public static final double slewRate = 20; // lower number for higher center of mass

    public static final class SwervePID {
      public static final double p = 0.01; //0.052
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class SwerveModules {

      // Front Left Module
      public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(
          8,
          1,
          true,
          true,
          false,
          1,
          //0.303 //apollo
          //.462 //swervebot
          0
          );

      // Front Right
      public static final SwerveModuleConfig frontRight= new SwerveModuleConfig(
          6,
          7,
          true,
          true,
          false,
          1,
          //0.966 //apollo
          //0 //swervebot 
          0
          );

      // Back Right
      public static final SwerveModuleConfig backRight = new SwerveModuleConfig(
          4,
          5,
          true,
          true,
          false,
          1,
          //0.485 //apollo
          //0.759 //swervebot
          0
          );

      // Back Left
      public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(
          2,
          3,
          true,
          true,
          false,
          1,
          //0.199 //apollo 
          //0.158 // swervebot
          //encoder offsets -> 0 to 1
          0
      );
    }

    public static final class ModuleLocations {
      public static final double dist = Units.inchesToMeters(11.0);
      public static final double robotRaduius = Math.sqrt(2 * Math.pow(dist, 2));
      public static final Translation2d frontLeft = new Translation2d(dist, dist);
      public static final Translation2d frontRight = new Translation2d(dist, -dist);
      public static final Translation2d backLeft = new Translation2d(-dist, dist);
      public static final Translation2d backRight = new Translation2d(-dist, -dist);
    }
  }

  public static final class AutoConstants {
    public static final class XPID {
      public static final double p = 1.5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class YPID {
      public static final double p = 1.5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class RPID {
      public static final double p = 0.0015;
      public static final double i = 0;
      public static final double d = 0.0002;
    }

    public static final int medianFilter = 5;
  }

  public static final class PathPlannerConstants {
    public static final class TranslationPID {
      public static final double p = 5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class RotationPID {
      public static final double p = 6;
      public static final double i = 0;
      public static final double d = 0;
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class CANdleConstants {
    public static final int id = 50;
    public static final int ledCount = 50;
  }
}
