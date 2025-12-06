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
import frc.robot.Constants.IntakeConstants;

/**
 * IntakeSubsystem controls the intake roller motor.
 */
public class IntakeSubsystem extends SubsystemBase {
  // Motor
  private final TalonSRX m_intakeMotor;

  // State
  private double m_currentSpeed = 0.0;

  // NetworkTables
  private final NetworkTable m_subsysTable;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor = new TalonSRX(IntakeConstants.kIntakeMotorId);

    // Initialize NetworkTables
    m_subsysTable = NetworkTableInstance.getDefault().getTable("Subsystems");
  }

  /**
   * Sets the intake motor speed.
   *
   * @param speed Motor speed (-1 to 1)
   */
  public void setSpeed(double speed) {
    m_currentSpeed = speed;
    m_intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  /** Runs the intake at the default intake speed. */
  public void intake() {
    setSpeed(IntakeConstants.kIntakeSpeed);
  }

  /** Runs the intake in reverse (outtake). */
  public void outtake() {
    setSpeed(-IntakeConstants.kIntakeSpeed);
  }

  /** Stops the intake motor. */
  public void stop() {
    setSpeed(0.0);
  }

  /** @return Current intake speed */
  public double getSpeed() {
    return m_currentSpeed;
  }

  /** @return true if intake is running */
  public boolean isRunning() {
    return Math.abs(m_currentSpeed) > 0.01;
  }

  @Override
  public void periodic() {
    // Publish to NetworkTables
    m_subsysTable.getEntry("Intake_Speed").setDouble(m_currentSpeed);
    m_subsysTable.getEntry("Intake_Active").setBoolean(isRunning());

    // SmartDashboard
    SmartDashboard.putNumber("Subsystems/Intake Speed", m_currentSpeed);
    SmartDashboard.putBoolean("Subsystems/Intake Active", isRunning());
  }
}
