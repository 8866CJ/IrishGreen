// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Default drive command that uses arcade drive with joystick inputs.
 */
public class DriveCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_speedSupplier;
  private final DoubleSupplier m_turnSupplier;

  /**
   * Creates a new DriveCommand.
   *
   * @param driveSubsystem The drive subsystem
   * @param speedSupplier  Supplier for forward/backward speed (-1 to 1)
   * @param turnSupplier   Supplier for rotation speed (-1 to 1)
   */
  public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier speedSupplier, DoubleSupplier turnSupplier) {
    m_driveSubsystem = driveSubsystem;
    m_speedSupplier = speedSupplier;
    m_turnSupplier = turnSupplier;

    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    m_driveSubsystem.arcadeDrive(m_speedSupplier.getAsDouble(), m_turnSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}
