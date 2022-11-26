// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShootConstants;

import java.lang.Math;

public class ManipulatorSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_topShootMotor = new WPI_TalonFX(ShootConstants.kTopShootMotorPort);
  private final WPI_TalonFX m_bottomShootMotor = new WPI_TalonFX(ShootConstants.kBottomShootMotorPort);
  
  // Solenoid for moving shooter arm up/down
  private final DoubleSolenoid m_armSolenoid = 
    new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, 
      ArmConstants.kArmSolenoidPorts[0], 
      ArmConstants.kArmSolenoidPorts[1]);

  // Solenoids for shoting balls
  private final DoubleSolenoid m_shootSolenoid1 =
    new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      ShootConstants.kShootSolenoid1Ports[0],
      ShootConstants.kShootSolenoid1Ports[1]);
  private final DoubleSolenoid m_shootSolenoid2 =
    new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      ShootConstants.kShootSolenoid2Ports[0],
      ShootConstants.kShootSolenoid2Ports[1]);
  
  public void configurePID(WPI_TalonFX motor) {
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.config_kP(0, ShootConstants.kShootMotor_kP);
    motor.config_kF(0, ShootConstants.kShootMotor_kF);
    motor.config_kD(0, ShootConstants.kShootMotor_kD);
    motor.config_kI(0, ShootConstants.kShootMotor_kI);
  }

  /** Creates a new ManipulatorSubsystem. */
  public ManipulatorSubsystem() {
    configurePID(m_topShootMotor);
    configurePID(m_bottomShootMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Velocity", m_topShootMotor.getSelectedSensorVelocity(1));
    SmartDashboard.putNumber("Bottom Velocity", m_bottomShootMotor.getSelectedSensorVelocity(1));
  }

  // Sets closed loop target velocity for shoot motors
  public void setVelocityPID(double setpoint) {
    m_topShootMotor.set(TalonFXControlMode.Velocity, setpoint);
    m_bottomShootMotor.set(TalonFXControlMode.Velocity, setpoint);
  }

  // Sets percentage output for shoot motors
  public void setVelocity(double velocity) {
    m_topShootMotor.set(velocity);
    m_bottomShootMotor.set(velocity);
  }

  public void raiseArm(){
    m_armSolenoid.set(Value.kReverse);
  }
  public void lowerArm(){
    m_armSolenoid.set(Value.kForward);
  }

  public DoubleSolenoid getShootSolenoid1(){
    return m_shootSolenoid1;
  }
  public DoubleSolenoid getShootSolenoid2(){
    return m_shootSolenoid2;
  }

  public void extendShootSolenoid(DoubleSolenoid solenoid){
    solenoid.set(Value.kForward);
  }
  public void retractShootSolenoid(DoubleSolenoid solenoid){
    solenoid.set(Value.kReverse);
  }

  public DoubleSolenoid.Value getArmStatus(){
    return m_armSolenoid.get();
  }

  // Returns if shoot motors are within the specified tolerance
  public boolean getShootMotorsReady(double setpoint){
    double topMotorError = Math.abs(setpoint - m_topShootMotor.getSelectedSensorVelocity());
    double bottomMotorError = Math.abs(setpoint - m_bottomShootMotor.getSelectedSensorVelocity());

    return (topMotorError < ShootConstants.kShootMotorSpeedTolerance) && (bottomMotorError < ShootConstants.kShootMotorSpeedTolerance);
  }
}
