// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  
  // Initializing drivetrain motor controllers
  private final WPI_TalonSRX m_leftMotor1 = new WPI_TalonSRX(DriveConstants.kLeftDriveMotor1Port);
  private final WPI_TalonSRX m_leftMotor2 = new WPI_TalonSRX(DriveConstants.kLeftDriveMotor2Port);
  private final WPI_TalonSRX m_rightMotor1 = new WPI_TalonSRX(DriveConstants.kRightDriveMotor1Port);
  private final WPI_TalonSRX m_rightMotor2 = new WPI_TalonSRX(DriveConstants.kRightDriveMotor2Port);

  // Grouping motor controllers
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
  
  // Initializing robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Motors on right side of robot are backwards by design
    m_rightMotors.setInverted(true);
  }
  
  // Drives the robot using tank controls
  public void tankDrive(double leftSpeed, double rightSpeed){
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }
}
