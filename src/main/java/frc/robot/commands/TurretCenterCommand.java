// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants.TurretConstants;

/**
 * Command to automatically align the turret to center (0 degrees).
 */
public class TurretCenterCommand extends Command {
  private final TurretSubsystem m_turretSubsystem;

  /**
   * Creates a new TurretCenterCommand.
   *
   * @param turretSubsystem The turret subsystem
   */
  public TurretCenterCommand(TurretSubsystem turretSubsystem) {
    m_turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    double angle = m_turretSubsystem.getAngle();

    if (Math.abs(angle) > TurretConstants.kAlignmentTolerance) {
      // Calculate speed to move toward center
      double speed = m_turretSubsystem.calculateAlignmentSpeed(-angle);
      m_turretSubsystem.setSpeed(speed);
    } else {
      m_turretSubsystem.stop();
    }
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
