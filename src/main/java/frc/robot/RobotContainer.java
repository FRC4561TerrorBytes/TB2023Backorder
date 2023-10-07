// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ScoreAlign;
import frc.robot.commands.autonomous.BalanceAuto;
import frc.robot.commands.autonomous.DriveUntilCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final SendableChooser<Supplier<Command>> m_autoChooser = new SendableChooser<>();
  private boolean isAuto;

  private final CommandXboxController m_primaryController = new CommandXboxController(0);
  private final CommandXboxController m_secondaryController = new CommandXboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.drive(
        modifyAxis(m_primaryController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(m_primaryController.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(m_primaryController.getRightX())
            * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.6,
        true),
        m_driveSubsystem));

    m_autoChooser.setDefaultOption("Do Nothing", () -> new DriveUntilCommand(m_driveSubsystem, 0.0, 0.0, () -> true));
    
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * 
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Driver nudges
    m_primaryController.povUp()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(-1.0, 0.0, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povDown()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(1.0, 0.0, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povLeft()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, -0.8, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povRight()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, 0.8, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.rightTrigger()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, 0.0, 1.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.leftTrigger()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, 0.0, -1.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));

    m_primaryController.x()
      .whileTrue(new ScoreAlign(m_driveSubsystem));
      // .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
  }

  public void changeAutoTrigger(boolean inAuto) {
    this.isAuto = inAuto;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    var autoCommandSupplier = m_autoChooser.getSelected();
    if (autoCommandSupplier != null) {
      return autoCommandSupplier.get();
    }
    return null;
  }

  public void teleopInit() {
    
  }

  public void autoInit() {
    
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value * 0.8;
  }
}
