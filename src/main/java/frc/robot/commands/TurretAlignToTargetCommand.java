// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to automatically align the turret to an AprilTag using the Limelight.
 */
public class TurretAlignToTargetCommand extends Command {
  private final TurretSubsystem m_turretSubsystem;

  /**
   * Creates a new TurretAlignToTargetCommand.
   *
   * @param turretSubsystem The turret subsystem
   */
  public TurretAlignToTargetCommand(TurretSubsystem turretSubsystem) {
    m_turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    if (m_turretSubsystem.hasTarget()) {
      double tx = m_turretSubsystem.getTargetTX();

      if (!m_turretSubsystem.isAligned(tx)) {
        // Calculate speed to align with target
        double speed = m_turretSubsystem.calculateAlignmentSpeed(tx);
        m_turretSubsystem.setSpeed(speed);
        SmartDashboard.putBoolean("Turret Aligned", false);
      } else {
        m_turretSubsystem.stop();
        SmartDashboard.putBoolean("Turret Aligned", true);
      }
    } else {
      // No target - stop turret
      m_turretSubsystem.stop();
      SmartDashboard.putBoolean("Turret Aligned", false);
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
