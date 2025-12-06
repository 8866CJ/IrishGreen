// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to run the intake. Also centers the turret first if needed.
 */
public class IntakeCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final TurretSubsystem m_turretSubsystem;

  /**
   * Creates a new IntakeCommand.
   *
   * @param intakeSubsystem The intake subsystem
   * @param turretSubsystem The turret subsystem (for centering check)
   */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_turretSubsystem = turretSubsystem;
    addRequirements(intakeSubsystem, turretSubsystem);
  }

  @Override
  public void execute() {
    // Check if turret needs to be centered first
    if (!m_turretSubsystem.isAtCenter()) {
      // Center the turret
      double angle = m_turretSubsystem.getAngle();
      double speed = m_turretSubsystem.calculateAlignmentSpeed(-angle);
      m_turretSubsystem.setSpeed(speed);
      m_intakeSubsystem.stop();
    } else {
      // Turret is centered, run intake
      m_turretSubsystem.stop();
      m_intakeSubsystem.intake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stop();
    m_turretSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Run while trigger is held
  }
}
