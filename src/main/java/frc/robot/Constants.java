// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    // CAN IDs
    public static final int kFrontLeftMotorId = 5;
    public static final int kBackLeftMotorId = 4;
    public static final int kFrontRightMotorId = 6;
    public static final int kBackRightMotorId = 7;

    // Robot physical constants
    public static final double kWheelbaseWidth = 0.57; // meters
    public static final double kMaxVelocity = 3.8; // meters per second
    public static final double kJoystickDeadband = 0.10; // 10% deadband

    // Speed multipliers
    public static final double kDriveSpeedMultiplier = 0.8;
    public static final double kTurnSpeedMultiplier = 0.20;

    // Field dimensions for boundary checking
    public static final double kFieldWidth = 16.46; // meters
    public static final double kFieldHeight = 8.23; // meters
    public static final double kRobotDiagonal = 1.01; // meters
  }

  public static class TurretConstants {
    // CAN ID
    public static final int kTurretMotorId = 8;

    // Turret limits and control
    public static final double kMaxRotation = 190.0; // degrees (±95 from center)
    public static final double kAlignmentTolerance = 2.0; // degrees
    public static final double kMinSpeed = 0.15;
    public static final double kMaxSpeed = 0.40;
    public static final double kManualSpeed = 0.2;
    public static final double kP = 0.02; // Proportional gain

    // Encoder constants
    public static final double kGearRatio = 100.0; // Motor rotations per turret rotation
    public static final double kDegreesPerMotorRotation = 360.0 / kGearRatio;
  }

  public static class IntakeConstants {
    // CAN ID
    public static final int kIntakeMotorId = 1;

    // Speeds
    public static final double kIntakeSpeed = 0.6;
  }

  public static class ShooterConstants {
    // CAN IDs
    public static final int kTransferMotorId = 9;
    public static final int kFlywheelMotorId = 2;
    public static final int kShooterMotorId = 3;

    // Flywheel constants
    public static final double kFlywheelTargetSpeed = 0.5; // 50% speed
    public static final double kFlywheelSpeedTolerance = 0.05; // 5% tolerance
    public static final double kFlywheelSpinupDelay = 0.5; // seconds
    public static final double kFlywheelAccelRate = 2.0;

    // Feed speeds
    public static final double kShooterFeedSpeed = 0.25;
    public static final double kTransferFeedSpeed = 0.25;
  }

  public static class VisionConstants {
    // Limelight name - use "" for default or specify like "limelight-turret"
    public static final String kLimelightName = "";
  }
}
