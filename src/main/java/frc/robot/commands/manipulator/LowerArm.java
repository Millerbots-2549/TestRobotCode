// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;

public class LowerArm extends CommandBase {
  private final ManipulatorSubsystem m_manipulatorSubsystem;

  /** Creates a new LowerArm. */
  public LowerArm(ManipulatorSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_manipulatorSubsystem = subsystem;
    addRequirements(m_manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulatorSubsystem.lowerArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_manipulatorSubsystem.setVelocity(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
