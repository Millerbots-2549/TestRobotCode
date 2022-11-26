// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ManipulatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootTwo extends SequentialCommandGroup {
  /** Creates a new ShootTwoLow. */
  public ShootTwo(ManipulatorSubsystem manip, double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RaiseArm(manip, speed).withTimeout(1),
      new SpinUp(manip, speed),
      new ShootBall(manip, manip.getShootSolenoid1(), speed).withTimeout(0.5),
      new SpinUp(manip, speed),
      new ShootBall(manip, manip.getShootSolenoid2(), speed).withTimeout(0.5));
  }
}
