// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.commands.auto.AutoShoot;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DriveFixedSpeed;
import frc.robot.commands.drive.SlowDrive;
import frc.robot.commands.manipulator.LowerArm;
import frc.robot.commands.manipulator.RaiseArm;
import frc.robot.commands.manipulator.ShootTwo;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ManipulatorSubsystem m_manipulatorSubsystem = new ManipulatorSubsystem();

  // Auto routines
  private final Command m_autoShoot = new AutoShoot(
    m_driveSubsystem, m_manipulatorSubsystem, AutoConstants.kAutoShootSpeed, 
    AutoConstants.kAutoDriveTime, AutoConstants.kAutoDriveSpeed);
  
  private final Command m_autoWait = new SequentialCommandGroup(
    new WaitCommand(AutoConstants.kAutoWaitTime),
    new DriveFixedSpeed(m_driveSubsystem, AutoConstants.kAutoDriveSpeed).withTimeout(AutoConstants.kAutoDriveTime));

  private final Command m_autoBail = new DriveFixedSpeed(
    m_driveSubsystem, AutoConstants.kAutoDriveSpeed).withTimeout(AutoConstants.kAutoDriveTime);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Initialize controllers
  XboxController m_driverController = new XboxController(IOConstants.kDriverControllerPort);
  XboxController m_manipulatorController = new XboxController(IOConstants.kManipulatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set default commands for subsystems
    // Sets default drive command to DefaultDrive, with driver controller's Left and Right Stick Y axis being the controls
    m_driveSubsystem.setDefaultCommand(new DefaultDrive(m_driveSubsystem, m_driverController::getLeftY, m_driverController::getRightY));
    // Sets default manipulator command to RaiseArm, which also spins up the shooter wheels at a low speed
    m_manipulatorSubsystem.setDefaultCommand(new RaiseArm(m_manipulatorSubsystem, ShootConstants.kShootLowSpeed));

    // Add commands to auto command chooser
    m_chooser.setDefaultOption("Shooting Auto", m_autoShoot);
    m_chooser.addOption("Wait Then Bail Auto", m_autoWait);
    m_chooser.addOption("Bail Auto", m_autoBail);

    // Put chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drives robot at lower speed when RB is held
    new JoystickButton(m_driverController, Button.kRightBumper.value).whileTrue(
      new SlowDrive(m_driveSubsystem, m_driverController::getLeftY, m_driverController::getRightY));
  
    // Lowers arm when A is pressed, defaults back to raised arm when A is pressed again
    new JoystickButton(m_driverController, Button.kA.value).toggleOnTrue(new LowerArm(m_manipulatorSubsystem));

    // Shoots two balls low when B is pressed, high when Y is pressed
    new JoystickButton(m_driverController, Button.kB.value).onTrue(new ShootTwo(m_manipulatorSubsystem, ShootConstants.kShootLowSpeed));
    new JoystickButton(m_driverController, Button.kY.value).onTrue(new ShootTwo(m_manipulatorSubsystem, ShootConstants.kShootHighSpeed));

    // Manipulator controller commands
    new JoystickButton(m_manipulatorController, Button.kA.value).toggleOnTrue(new LowerArm(m_manipulatorSubsystem));
    new JoystickButton(m_manipulatorController, Button.kB.value).onTrue(new ShootTwo(m_manipulatorSubsystem, ShootConstants.kShootLowSpeed));
    new JoystickButton(m_manipulatorController, Button.kY.value).onTrue(new ShootTwo(m_manipulatorSubsystem, ShootConstants.kShootHighSpeed));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
