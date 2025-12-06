// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CancelShootCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurretAlignToTargetCommand;
import frc.robot.commands.TurretCenterCommand;
import frc.robot.commands.TurretManualCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure default commands
    configureDefaultCommands();
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Configures default commands for subsystems.
   */
  private void configureDefaultCommands() {
    // Default drive command - arcade drive with left stick Y for speed, right stick X for turn
    m_driveSubsystem.setDefaultCommand(
        new DriveCommand(
            m_driveSubsystem,
            () -> -m_driverController.getLeftY(),  // Invert for forward = positive
            () -> -m_driverController.getRightX() // Invert for right = positive turn
        )
    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // ==================== Intake Controls ====================
    // Left Trigger - Run intake (centers turret first if needed)
    m_driverController.leftTrigger(0.7)
        .whileTrue(new IntakeCommand(m_intakeSubsystem, m_turretSubsystem));

    // ==================== Shooter Controls ====================
    // Right Trigger - Shooter sequence (spin up, wait, feed)
    m_driverController.rightTrigger(0.7)
        .whileTrue(new ShootCommand(m_shooterSubsystem));

    // B Button - Cancel shooter
    m_driverController.b()
        .onTrue(new CancelShootCommand(m_shooterSubsystem));

    // ==================== Turret Controls ====================
    // Left Bumper - Center turret
    m_driverController.leftBumper()
        .whileTrue(new TurretCenterCommand(m_turretSubsystem));

    // Right Bumper - Align to AprilTag
    m_driverController.rightBumper()
        .whileTrue(new TurretAlignToTargetCommand(m_turretSubsystem));

    // D-Pad Right (POV 90) - Manual turret right
    m_driverController.pov(90)
        .whileTrue(new TurretManualCommand(m_turretSubsystem, 1.0));

    // D-Pad Left (POV 270) - Manual turret left
    m_driverController.pov(270)
        .whileTrue(new TurretManualCommand(m_turretSubsystem, -1.0));

    // ==================== Utility ====================
    // A Button - Reset odometry
    m_driverController.a()
        .onTrue(Commands.runOnce(() -> m_driveSubsystem.resetOdometry()));

    // X Button - Reset turret encoder (use when physically centered)
    m_driverController.x()
        .onTrue(Commands.runOnce(() -> m_turretSubsystem.resetEncoder()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // No autonomous command for now - return null or a simple command
    return Commands.print("No autonomous command configured");
  }

  /**
   * Called periodically from Robot.robotPeriodic() to update dashboard info.
   */
  public void updateDashboard() {
    // Update controller state on dashboard
    SmartDashboard.putBoolean("Subsystems/Left Trigger Down", m_driverController.getLeftTriggerAxis() > 0.7);
    SmartDashboard.putBoolean("Subsystems/Right Trigger Down", m_driverController.getRightTriggerAxis() > 0.7);
    SmartDashboard.putBoolean("Subsystems/A Button", m_driverController.a().getAsBoolean());
  }
}
