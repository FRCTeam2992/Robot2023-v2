// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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
  public static class DriveConstants {
    // Drive Variables
    public static final boolean isFieldCentric = true;
    public static final boolean isVelocityControlled = true;
    public static final boolean isGyroCorrected = true;
    public static final double joystickDeadband = 0.15;
    public static double joystickXYSmoothFactor = 0.5;
    public static double joystickRotationSmoothFactor = 0.5;
    public static double joystickRotationInverseDeadband = 0.14;

    // Length and Width of the Robot in Meters (Inches: 22.0 x 24.5)
    public static final double swerveWidth = 0.6604;
    public static final double swerveLength = 0.6604;

    // Max Swerve Speed (Velocity Control)
    public static final double swerveMaxSpeed = 4.5; // (Meters per Second)(2 Slow, 4.5 normal)

    // Swerve Wheels and Gear Ratio
    public static final double driveGearRatio = 6.75;// 6.75:1
    public static final double driveWheelDiameter = 0.098552;

    // Analog Encoder Offsets (Degrees) - Opposite of Raw Reading - Bevel Gear to
    // Right
    public static final double frontLeftOffset = 150.1;
    public static final double frontRightOffset = 24.4;
    public static final double rearLeftOffset = -163.8;
    public static final double rearRightOffset = 6.8;

    public static class PIDConstants {
      // Swerve Drive PID (Velocity Control)
      public static final double driveP = 0.05;// .05
      public static final double driveI = 0.0;// .0
      public static final double driveD = 0.01;
      public static final double driveF = 0.047;

      // Swerve Turn PIDs
      public static final double turnP = 0.013; // .013
      public static final double turnI = 0.0;// .0
      public static final double turnD = 0.00005;
    }

    // Gyro P
    public static final double driveGyroP = 0.005;

    // Swerve Module Translations x=.591/2 y=.654/2
    public static final Translation2d frontLeftLocation = new Translation2d(0.2955, 0.327);
    public static final Translation2d frontRightLocation = new Translation2d(0.2955, -0.327);
    public static final Translation2d rearLeftLocation = new Translation2d(-0.2955, 0.327);
    public static final Translation2d rearRightLocation = new Translation2d(-0.2955, -0.327);

    // Swerve X Axis Correction PID (Path Following)
    public static final double xCorrectionP = 5.0;
    public static final double xCorrectionI = 0.0;
    public static final double xCorrectionD = 0.0;

    // Swerve Y Axis Correction PID (Path Following)
    public static final double yCorrectionP = 5.0;
    public static final double yCorrectionI = 0.0;
    public static final double yCorrectionD = 0.0;

    // Swerve Theta Axis Correction PID (Path Following)
    public static final double thetaCorrectionP = 3.0;
    public static final double thetaCorrectionI = 0.0;
    public static final double thetaCorrectionD = 0.0;

    // Max Path Following Drive Speeds
    public static final double maxPathFollowingVelocity = 3.0; // (Meters per Second)
    public static final double maxPathFollowingAcceleration = 2; // (Meters per Second Squared)

    // Max Path Following Turn Speeds
    public static final double maxThetaVelocity = 6.28; // (Radians per Second)
    public static final double maxThetaAcceleration = 6.28; // (Radians per Second Squared)

    // Max speeds where its safe to X wheels
    public static final double maxSpeedToX = 0.25; // m/sec
    public static final double maxTurnToX = 20.0; // degrees/sec

    public static class CanIDs {
      public static int frontLeftDrive = 4;
      public static int frontLeftTurn = 5;
      public static int frontRightDrive = 2;
      public static int frontRightTurn = 3;
      public static int rearLeftDrive = 8;
      public static int rearLeftTurn = 9;
      public static int rearRightDrive = 6;
      public static int rearRightTurn = 7;

      public static int frontLeftEncoder = 5;
      public static int frontRightEncoder = 3;
      public static int rearLeftEncoder = 9;
      public static int rearRightEncoder = 7;
    }

  }

  public static class IntakeConstants {
    public static class CanIDs{
      public static int intakeMotor = 22;
    }
  }

  public static class ElevatorConstants {
    public static class CanIDs{
      public static int elevatorMotor = 25;
    }

  }

  public static class SpindexerConstants {
    public static class CanIDs{
      public static int spindexerMotor = 23;
    }

  }

  public static class ArmConstants {
    public static class CanIDs{
      public static int armMotor = 26;
    }

  }


  public static class DrivetrainConstants {

  }
}