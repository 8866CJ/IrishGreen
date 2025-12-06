// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to run the shooter sequence (spin up, wait, feed).
 */
public class ShootCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private boolean m_started = false;

  /**
   * Creates a new ShootCommand.
   *
   * @param shooterSubsystem The shooter subsystem
   */
  public ShootCommand(ShooterSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    m_started = false;
  }

  @Override
  public void execute() {
    // Start shooting sequence on first call
    if (!m_started && m_shooterSubsystem.isIdle()) {
      m_shooterSubsystem.startShooting();
      m_started = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Cancel shooting when command ends (button released)
    m_shooterSubsystem.cancelShooting();
  }

  @Override
  public boolean isFinished() {
    return false; // Run while trigger is held
  }
}
