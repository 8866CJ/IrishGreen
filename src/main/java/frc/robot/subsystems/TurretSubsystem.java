// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

/**
 * TurretSubsystem controls the turret rotation and provides Limelight integration.
 */
public class TurretSubsystem extends SubsystemBase {
  // Motor and encoder
  private final SparkMax m_turretMotor;
  private final RelativeEncoder m_encoder;

  // NetworkTables
  private final NetworkTable m_visionTable;
  private final NetworkTable m_subsysTable;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    // Initialize motor
    m_turretMotor = new SparkMax(TurretConstants.kTurretMotorId, MotorType.kBrushless);
    m_encoder = m_turretMotor.getEncoder();

    // Configure encoder to return degrees directly using SparkMaxConfig (REVLib 2025 API)
    SparkMaxConfig config = new SparkMaxConfig();
    config.encoder.positionConversionFactor(TurretConstants.kDegreesPerMotorRotation);
    m_turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // Reset encoder position - assume turret starts centered
    m_encoder.setPosition(0.0);

    // Initialize NetworkTables
    m_visionTable = NetworkTableInstance.getDefault().getTable("Vision");
    m_subsysTable = NetworkTableInstance.getDefault().getTable("Subsystems");
  }

  /**
   * Sets the turret motor speed with soft limits applied.
   *
   * @param speed Motor speed (-1 to 1)
   */
  public void setSpeed(double speed) {
    double angle = getAngle();

    // Apply soft limits
    if (angle > TurretConstants.kMaxRotation / 2.0 && speed > 0) {
      speed = 0.0;
      SmartDashboard.putString("Turret Limit", "POSITIVE LIMIT");
    } else if (angle < -TurretConstants.kMaxRotation / 2.0 && speed < 0) {
      speed = 0.0;
      SmartDashboard.putString("Turret Limit", "NEGATIVE LIMIT");
    } else {
      SmartDashboard.putString("Turret Limit", "OK");
    }

    m_turretMotor.set(speed);
  }

  /** Stops the turret motor. */
  public void stop() {
    m_turretMotor.set(0.0);
  }

  /**
   * Gets the current turret angle from the encoder.
   *
   * @return Angle in degrees (0 = center, positive = right, negative = left)
   */
  public double getAngle() {
    return m_encoder.getPosition();
  }

  /**
   * Checks if the turret is at center position.
   *
   * @return true if within tolerance of center
   */
  public boolean isAtCenter() {
    return Math.abs(getAngle()) < TurretConstants.kAlignmentTolerance;
  }

  /**
   * Checks if the turret is aligned with a target.
   *
   * @param targetOffset The offset to the target in degrees
   * @return true if within alignment tolerance
   */
  public boolean isAligned(double targetOffset) {
    return Math.abs(targetOffset) < TurretConstants.kAlignmentTolerance;
  }

  /** Resets the encoder position to zero. Call when turret is physically centered. */
  public void resetEncoder() {
    m_encoder.setPosition(0.0);
  }

  /**
   * Calculates the speed needed to move toward a target angle using P control.
   *
   * @param error The angular error in degrees
   * @return The motor speed to apply
   */
  public double calculateAlignmentSpeed(double error) {
    double speed = error * TurretConstants.kP;

    // Clamp to min/max speeds
    if (speed > 0) {
      speed = Math.max(TurretConstants.kMinSpeed, Math.min(TurretConstants.kMaxSpeed, speed));
    } else if (speed < 0) {
      speed = Math.max(-TurretConstants.kMaxSpeed, Math.min(-TurretConstants.kMinSpeed, speed));
    }

    return speed;
  }

  // ==================== Limelight Methods ====================

  /** @return true if Limelight has a valid target */
  public boolean hasTarget() {
    return LimelightHelpers.getTV(VisionConstants.kLimelightName);
  }

  /** @return Horizontal offset to target in degrees */
  public double getTargetTX() {
    return LimelightHelpers.getTX(VisionConstants.kLimelightName);
  }

  /** @return Vertical offset to target in degrees */
  public double getTargetTY() {
    return LimelightHelpers.getTY(VisionConstants.kLimelightName);
  }

  /** @return Target area (0-100% of image) */
  public double getTargetArea() {
    return LimelightHelpers.getTA(VisionConstants.kLimelightName);
  }

  /** @return AprilTag fiducial ID, or -1 if none */
  public double getFiducialID() {
    return LimelightHelpers.getFiducialID(VisionConstants.kLimelightName);
  }

  @Override
  public void periodic() {
    double angle = getAngle();

    // Publish turret data
    m_subsysTable.getEntry("Turret_Angle").setDouble(angle);
    m_subsysTable.getEntry("Turret_Encoder_Position").setDouble(m_encoder.getPosition());
    m_subsysTable.getEntry("Turret_Encoder_Velocity").setDouble(m_encoder.getVelocity());

    // Publish vision data
    m_visionTable.getEntry("Turret_Angle").setDouble(angle);
    m_visionTable.getEntry("Timestamp").setDouble(edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
    m_visionTable.getEntry("Limelight_Pipeline_Latency")
        .setDouble(LimelightHelpers.getLatency_Pipeline(VisionConstants.kLimelightName));
    m_visionTable.getEntry("Limelight_Capture_Latency")
        .setDouble(LimelightHelpers.getLatency_Capture(VisionConstants.kLimelightName));

    // SmartDashboard
    SmartDashboard.putNumber("Turret Angle", angle);
    SmartDashboard.putNumber("Turret Encoder Raw", m_encoder.getPosition());
    SmartDashboard.putBoolean("Turret At Zero", isAtCenter());

    // Limelight data
    if (hasTarget()) {
      SmartDashboard.putNumber("Limelight TX", getTargetTX());
      SmartDashboard.putNumber("Limelight TY", getTargetTY());
      SmartDashboard.putNumber("Limelight TA", getTargetArea());
      SmartDashboard.putNumber("AprilTag ID", getFiducialID());
      SmartDashboard.putString("AprilTag Status", "Target Acquired");

      m_visionTable.getEntry("HasTarget").setBoolean(true);
      m_visionTable.getEntry("Target_TX").setDouble(getTargetTX());
      m_visionTable.getEntry("Target_TY").setDouble(getTargetTY());
      m_visionTable.getEntry("Target_TA").setDouble(getTargetArea());
      m_visionTable.getEntry("Target_ID").setDouble(getFiducialID());
    } else {
      SmartDashboard.putString("AprilTag Status", "No Target");
      m_visionTable.getEntry("HasTarget").setBoolean(false);
    }
  }
}
