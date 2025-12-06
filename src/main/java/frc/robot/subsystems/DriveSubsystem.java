// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * DriveSubsystem controls the robot's drivetrain and handles odometry.
 */
public class DriveSubsystem extends SubsystemBase {
  // Motors
  private final TalonSRX m_frontLeft;
  private final TalonSRX m_backLeft;
  private final TalonSRX m_frontRight;
  private final TalonSRX m_backRight;

  // Odometry state
  private double m_x = 0.0; // meters
  private double m_y = 0.0; // meters
  private double m_theta = 0.0; // radians
  private double m_prevTime = 0.0;
  private double m_lastLeftSpeed = 0.0;
  private double m_lastRightSpeed = 0.0;

  // NetworkTables
  private final NetworkTable m_poseTable;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_frontLeft = new TalonSRX(DriveConstants.kFrontLeftMotorId);
    m_backLeft = new TalonSRX(DriveConstants.kBackLeftMotorId);
    m_frontRight = new TalonSRX(DriveConstants.kFrontRightMotorId);
    m_backRight = new TalonSRX(DriveConstants.kBackRightMotorId);

    // Set followers
    m_backLeft.follow(m_frontLeft);
    m_backRight.follow(m_frontRight);

    // Initialize NetworkTables
    m_poseTable = NetworkTableInstance.getDefault().getTable("Pose");

    // Reset odometry
    resetOdometry();
  }

  /**
   * Drives the robot with arcade drive controls.
   *
   * @param speed Forward/backward speed (-1 to 1)
   * @param turn  Rotation speed (-1 to 1)
   */
  public void arcadeDrive(double speed, double turn) {
    // Apply deadband
    speed = applyDeadband(speed);
    turn = applyDeadband(turn) * DriveConstants.kTurnSpeedMultiplier;

    // Calculate motor speeds
    double leftSpeed = speed - turn;
    double rightSpeed = speed + turn;

    // Clamp values
    leftSpeed = Math.max(-1, Math.min(1, leftSpeed));
    rightSpeed = Math.max(-1, Math.min(1, rightSpeed));

    // Apply speed multiplier and set motors
    m_frontLeft.set(ControlMode.PercentOutput, leftSpeed * DriveConstants.kDriveSpeedMultiplier);
    m_frontRight.set(ControlMode.PercentOutput, rightSpeed * DriveConstants.kDriveSpeedMultiplier);

    // Store for odometry
    m_lastLeftSpeed = leftSpeed * DriveConstants.kDriveSpeedMultiplier;
    m_lastRightSpeed = rightSpeed * DriveConstants.kDriveSpeedMultiplier;
  }

  /** Stops all drive motors. */
  public void stop() {
    m_frontLeft.set(ControlMode.PercentOutput, 0);
    m_frontRight.set(ControlMode.PercentOutput, 0);
    m_lastLeftSpeed = 0;
    m_lastRightSpeed = 0;
  }

  /**
   * Applies a deadband to a value.
   *
   * @param value The value to apply deadband to
   * @return The value with deadband applied
   */
  private double applyDeadband(double value) {
    if (Math.abs(value) < DriveConstants.kJoystickDeadband) {
      return 0.0;
    }
    return value;
  }

  /** Resets the odometry to origin. */
  public void resetOdometry() {
    m_x = 0.0;
    m_y = 0.0;
    m_theta = 0.0;
    m_prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
  }

  /**
   * Resets the odometry to a specific pose.
   *
   * @param x     X position in meters
   * @param y     Y position in meters
   * @param theta Heading in radians
   */
  public void resetOdometry(double x, double y, double theta) {
    m_x = x;
    m_y = y;
    m_theta = theta;
    m_prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
  }

  /** @return Current X position in meters */
  public double getX() {
    return m_x;
  }

  /** @return Current Y position in meters */
  public double getY() {
    return m_y;
  }

  /** @return Current heading in radians */
  public double getTheta() {
    return m_theta;
  }

  /** @return Current heading in degrees */
  public double getThetaDegrees() {
    return Math.toDegrees(m_theta);
  }

  @Override
  public void periodic() {
    // Update odometry
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double dt = currentTime - m_prevTime;
    m_prevTime = currentTime;

    // Calculate velocities from motor speeds
    double vLeft = m_lastLeftSpeed * DriveConstants.kMaxVelocity;
    double vRight = m_lastRightSpeed * DriveConstants.kMaxVelocity;

    // Differential drive kinematics
    double v = (vLeft + vRight) / 2.0; // Linear velocity (m/s)
    double omega = (vRight - vLeft) / DriveConstants.kWheelbaseWidth; // Angular velocity (rad/s)

    // Update pose (Euler integration with midpoint method)
    double dtheta = omega * dt;
    double avgTheta = m_theta + dtheta / 2.0;

    double dx = v * Math.cos(avgTheta) * dt;
    double dy = v * Math.sin(avgTheta) * dt;

    m_x += dx;
    m_y += dy;
    m_theta += dtheta;

    // Apply boundary checking
    double halfDiagonal = DriveConstants.kRobotDiagonal / 2;
    m_x = Math.max(halfDiagonal, Math.min(DriveConstants.kFieldWidth - halfDiagonal, m_x));
    m_y = Math.max(halfDiagonal, Math.min(DriveConstants.kFieldHeight - halfDiagonal, m_y));

    // Normalize angle to [-PI, PI]
    m_theta = Math.atan2(Math.sin(m_theta), Math.cos(m_theta));

    // Calculate velocity components
    double vx = v * Math.cos(m_theta);
    double vy = v * Math.sin(m_theta);

    // Publish to NetworkTables
    m_poseTable.getEntry("X").setDouble(m_x);
    m_poseTable.getEntry("Y").setDouble(m_y);
    m_poseTable.getEntry("Theta").setDouble(m_theta);
    m_poseTable.getEntry("VX").setDouble(vx);
    m_poseTable.getEntry("VY").setDouble(vy);
    m_poseTable.getEntry("Omega").setDouble(omega);

    // Publish to SmartDashboard
    SmartDashboard.putNumber("Odometry X", m_x);
    SmartDashboard.putNumber("Odometry Y", m_y);
    SmartDashboard.putNumber("Odometry Theta (deg)", Math.toDegrees(m_theta));
    SmartDashboard.putNumber("Velocity VX", vx);
    SmartDashboard.putNumber("Velocity VY", vy);
    SmartDashboard.putNumber("Rotational Velocity (rad/s)", omega);
  }
}
