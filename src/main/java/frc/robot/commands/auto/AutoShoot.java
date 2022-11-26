// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveFixedSpeed;
import frc.robot.commands.manipulator.RaiseArm;
import frc.robot.commands.manipulator.ShootBall;
import frc.robot.commands.manipulator.SpinUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new AutoSShoot. */
  public AutoShoot(DriveSubsystem drive, ManipulatorSubsystem manip, double shootSpeed, double driveTime, double driveSpeed) { 
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RaiseArm(manip, shootSpeed).withTimeout(1),
      new SpinUp(manip, shootSpeed),
      new ShootBall(manip, manip.getShootSolenoid1(), shootSpeed).withTimeout(0.5),
      new ShootBall(manip, manip.getShootSolenoid2(), shootSpeed).withTimeout(0.5),
      new DriveFixedSpeed(drive, driveSpeed).withTimeout(driveTime));
  }
}
