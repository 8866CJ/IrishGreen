// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants.TurretConstants;

/**
 * Command for manual turret control using D-pad.
 */
public class TurretManualCommand extends Command {
  private final TurretSubsystem m_turretSubsystem;
  private final double m_direction; // 1.0 for right, -1.0 for left

  /**
   * Creates a new TurretManualCommand.
   *
   * @param turretSubsystem The turret subsystem
   * @param direction       Direction to rotate (1.0 = right, -1.0 = left)
   */
  public TurretManualCommand(TurretSubsystem turretSubsystem, double direction) {
    m_turretSubsystem = turretSubsystem;
    m_direction = direction;
    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    m_turretSubsystem.setSpeed(TurretConstants.kManualSpeed * m_direction);
  }

  @Override
  public void end(boolean interrupted) {
    m_turretSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Run while button is held
  }
}
