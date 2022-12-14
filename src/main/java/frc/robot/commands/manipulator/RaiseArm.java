// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;

public class RaiseArm extends CommandBase {
  private final ManipulatorSubsystem m_manipulatorSubsystem;
  private final double m_shootSpeed;

  /** Creates a new RaiseArm. */
  public RaiseArm(ManipulatorSubsystem subsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_manipulatorSubsystem = subsystem;
    m_shootSpeed = speed;

    addRequirements(m_manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulatorSubsystem.raiseArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_manipulatorSubsystem.setVelocityPID(m_shootSpeed);
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
