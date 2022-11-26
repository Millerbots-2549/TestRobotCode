// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ShootBall extends CommandBase {
  private final ManipulatorSubsystem m_manipulatorSubsystem;
  private final DoubleSolenoid m_shootSolenoid;
  private final double m_shootSpeed;

  /** Creates a new ShootOne. */
  public ShootBall(ManipulatorSubsystem subsystem, DoubleSolenoid solenoid, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_manipulatorSubsystem = subsystem;
    m_shootSolenoid = solenoid;
    m_shootSpeed = speed;

    addRequirements(m_manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulatorSubsystem.extendShootSolenoid(m_shootSolenoid);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_manipulatorSubsystem.setVelocityPID(m_shootSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_manipulatorSubsystem.retractShootSolenoid(m_shootSolenoid);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
