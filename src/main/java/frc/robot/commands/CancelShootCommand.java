// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to cancel/stop the shooter sequence.
 */
public class CancelShootCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;

  /**
   * Creates a new CancelShootCommand.
   *
   * @param shooterSubsystem The shooter subsystem
   */
  public CancelShootCommand(ShooterSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.cancelShooting();
  }

  @Override
  public boolean isFinished() {
    return true; // Instant command
  }
}
