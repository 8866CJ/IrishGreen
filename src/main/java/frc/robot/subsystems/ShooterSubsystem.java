// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

/**
 * ShooterSubsystem controls the flywheel, shooter feed, and transfer motors.
 * Implements a state machine for the shooting sequence.
 */
public class ShooterSubsystem extends SubsystemBase {
  // Motors
  private final TalonSRX m_flywheelMotor;
  private final TalonSRX m_shooterMotor;
  private final SparkMax m_transferMotor;

  // State machine
  public enum ShooterState {
    IDLE,
    SPINNING_UP,
    WAITING,
    FEEDING,
    STOPPING
  }

  private ShooterState m_state = ShooterState.IDLE;
  private double m_stateStartTime = 0.0;
  private double m_prevTime = 0.0;

  // Motor speeds
  private double m_flywheelSpeed = 0.0;
  private double m_shooterSpeed = 0.0;
  private double m_transferSpeed = 0.0;

  // NetworkTables
  private final NetworkTable m_subsysTable;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_flywheelMotor = new TalonSRX(ShooterConstants.kFlywheelMotorId);
    m_shooterMotor = new TalonSRX(ShooterConstants.kShooterMotorId);
    m_transferMotor = new SparkMax(TurretConstants.kTurretMotorId, MotorType.kBrushless);

    // Initialize NetworkTables
    m_subsysTable = NetworkTableInstance.getDefault().getTable("Subsystems");

    m_prevTime = Timer.getFPGATimestamp();
  }

  // ==================== State Machine Control ====================

  /** Starts the shooting sequence. */
  public void startShooting() {
    if (m_state == ShooterState.IDLE) {
      m_state = ShooterState.SPINNING_UP;
      m_stateStartTime = Timer.getFPGATimestamp();
    }
  }

  /** Cancels the shooting sequence. */
  public void cancelShooting() {
    if (m_state != ShooterState.IDLE) {
      m_state = ShooterState.STOPPING;
    }
  }

  /** Forces the shooter to stop immediately. */
  public void forceStop() {
    m_state = ShooterState.IDLE;
    m_flywheelSpeed = 0.0;
    m_shooterSpeed = 0.0;
    m_transferSpeed = 0.0;
    updateMotors();
  }

  /** @return Current shooter state */
  public ShooterState getState() {
    return m_state;
  }

  /** @return true if shooter is idle */
  public boolean isIdle() {
    return m_state == ShooterState.IDLE;
  }

  /** @return true if flywheel is at target speed */
  public boolean isFlywheelAtSpeed() {
    return Math.abs(m_flywheelSpeed - ShooterConstants.kFlywheelTargetSpeed) 
           < ShooterConstants.kFlywheelSpeedTolerance;
  }

  /** @return true if currently feeding game pieces */
  public boolean isFeeding() {
    return m_state == ShooterState.FEEDING;
  }

  // ==================== Direct Motor Control ====================

  /**
   * Sets the transfer motor speed directly.
   *
   * @param speed Motor speed (-1 to 1)
   */
  public void setTransferSpeed(double speed) {
    m_transferSpeed = speed;
    m_transferMotor.set(m_transferSpeed);
  }

  /** @return Current flywheel speed */
  public double getFlywheelSpeed() {
    return m_flywheelSpeed;
  }

  /** @return Current shooter feed speed */
  public double getShooterSpeed() {
    return m_shooterSpeed;
  }

  /** @return Current transfer speed */
  public double getTransferSpeed() {
    return m_transferSpeed;
  }

  // ==================== Internal Methods ====================

  /** Updates the physical motor outputs based on current speed values. */
  private void updateMotors() {
    m_flywheelMotor.set(ControlMode.PercentOutput, m_flywheelSpeed);
    m_shooterMotor.set(ControlMode.PercentOutput, m_shooterSpeed);
    m_transferMotor.set(m_transferSpeed);
  }

  /** Runs the state machine logic. */
  private void runStateMachine() {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - m_prevTime;
    m_prevTime = currentTime;

    switch (m_state) {
      case IDLE:
        m_flywheelSpeed = 0.0;
        m_shooterSpeed = 0.0;
        m_transferSpeed = 0.0;
        break;

      case SPINNING_UP:
        // Ramp up flywheel
        if (m_flywheelSpeed < ShooterConstants.kFlywheelTargetSpeed) {
          m_flywheelSpeed += ShooterConstants.kFlywheelAccelRate * dt;
          if (m_flywheelSpeed > ShooterConstants.kFlywheelTargetSpeed) {
            m_flywheelSpeed = ShooterConstants.kFlywheelTargetSpeed;
          }
        }

        // Check if at speed
        if (isFlywheelAtSpeed()) {
          m_state = ShooterState.WAITING;
          m_stateStartTime = currentTime;
        }
        break;

      case WAITING:
        // Maintain flywheel speed
        m_flywheelSpeed = ShooterConstants.kFlywheelTargetSpeed;

        // Wait for spinup delay
        if (currentTime - m_stateStartTime >= ShooterConstants.kFlywheelSpinupDelay) {
          m_state = ShooterState.FEEDING;
        }
        break;

      case FEEDING:
        // Maintain flywheel and activate feed motors
        m_flywheelSpeed = ShooterConstants.kFlywheelTargetSpeed;
        m_shooterSpeed = ShooterConstants.kShooterFeedSpeed;
        m_transferSpeed = ShooterConstants.kTransferFeedSpeed;
        break;

      case STOPPING:
        // Ramp down flywheel
        if (m_flywheelSpeed > 0.0) {
          m_flywheelSpeed -= ShooterConstants.kFlywheelAccelRate * dt;
          if (m_flywheelSpeed < 0.0) {
            m_flywheelSpeed = 0.0;
          }
        }

        m_shooterSpeed = 0.0;
        m_transferSpeed = 0.0;

        // Return to idle when stopped
        if (m_flywheelSpeed <= 0.0) {
          m_state = ShooterState.IDLE;
        }
        break;
    }

    updateMotors();
  }

  @Override
  public void periodic() {
    // Run state machine
    runStateMachine();

    // Calculate derived values
    double timeInState = Timer.getFPGATimestamp() - m_stateStartTime;
    double flywheelSpeedPercent = (m_flywheelSpeed / ShooterConstants.kFlywheelTargetSpeed) * 100.0;

    // Publish to NetworkTables - State
    m_subsysTable.getEntry("Shooter_System/State").setString(m_state.toString());
    m_subsysTable.getEntry("Shooter_System/State_Time").setDouble(timeInState);
    m_subsysTable.getEntry("Shooter_System/Is_Idle").setBoolean(m_state == ShooterState.IDLE);
    m_subsysTable.getEntry("Shooter_System/Is_Spinning_Up").setBoolean(m_state == ShooterState.SPINNING_UP);
    m_subsysTable.getEntry("Shooter_System/Is_Waiting").setBoolean(m_state == ShooterState.WAITING);
    m_subsysTable.getEntry("Shooter_System/Is_Feeding").setBoolean(m_state == ShooterState.FEEDING);
    m_subsysTable.getEntry("Shooter_System/Is_Stopping").setBoolean(m_state == ShooterState.STOPPING);

    // Publish to NetworkTables - Flywheel
    m_subsysTable.getEntry("Shooter_System/Flywheel_Speed").setDouble(m_flywheelSpeed);
    m_subsysTable.getEntry("Shooter_System/Flywheel_Target").setDouble(ShooterConstants.kFlywheelTargetSpeed);
    m_subsysTable.getEntry("Shooter_System/Flywheel_Speed_Percent").setDouble(flywheelSpeedPercent);
    m_subsysTable.getEntry("Shooter_System/Flywheel_At_Speed").setBoolean(isFlywheelAtSpeed());
    m_subsysTable.getEntry("Shooter_System/Flywheel_Error")
        .setDouble(ShooterConstants.kFlywheelTargetSpeed - m_flywheelSpeed);

    // Publish to NetworkTables - Feed
    m_subsysTable.getEntry("Shooter_System/Shooter_Feed_Speed").setDouble(m_shooterSpeed);
    m_subsysTable.getEntry("Shooter_System/Transfer_Feed_Speed").setDouble(m_transferSpeed);
    m_subsysTable.getEntry("Shooter_System/Is_Feeding_Active").setBoolean(m_shooterSpeed > 0.0 || m_transferSpeed > 0.0);

    // Publish general subsystem data
    m_subsysTable.getEntry("Shooter_Speed").setDouble(m_shooterSpeed);
    m_subsysTable.getEntry("Transfer_Speed").setDouble(m_transferSpeed);
    m_subsysTable.getEntry("Flywheel_Speed").setDouble(m_flywheelSpeed);
    m_subsysTable.getEntry("Shooter_State").setString(m_state.toString());
    m_subsysTable.getEntry("Shooter_Active").setBoolean(m_shooterSpeed > 0.01);
    m_subsysTable.getEntry("Transfer_Active").setBoolean(m_transferSpeed > 0.01);
    m_subsysTable.getEntry("Flywheel_Active").setBoolean(m_flywheelSpeed > 0.01);

    // SmartDashboard
    SmartDashboard.putNumber("Subsystems/Shooter Speed", m_shooterSpeed);
    SmartDashboard.putNumber("Subsystems/Transfer Speed", m_transferSpeed);
    SmartDashboard.putNumber("Subsystems/Flywheel Speed", m_flywheelSpeed);
    SmartDashboard.putString("Subsystems/Shooter State", m_state.toString());
    SmartDashboard.putNumber("Subsystems/Flywheel Target", ShooterConstants.kFlywheelTargetSpeed);
    SmartDashboard.putBoolean("Subsystems/Flywheel At Speed", isFlywheelAtSpeed());
    SmartDashboard.putNumber("Subsystems/State Time", timeInState);
    SmartDashboard.putNumber("Subsystems/Flywheel Speed Percent", flywheelSpeedPercent);
  }
}
