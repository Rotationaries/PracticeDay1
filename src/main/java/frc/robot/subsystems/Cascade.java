// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CascadeConstants;

public class Cascade extends SubsystemBase {

  public static CANSparkMax motor1 = new CANSparkMax(CascadeConstants.kCascadeMotor1Port, MotorType.kBrushless);
  public static CANSparkMax motor2 = new CANSparkMax(CascadeConstants.kCascadeMotor1Port, MotorType.kBrushless);

  MotorControllerGroup m_cascadeMotors = new MotorControllerGroup(motor1, motor2);
  
  private final RelativeEncoder m_encoder1 = motor1.getEncoder();
  
  private final RelativeEncoder m_encoder2 = motor2.getEncoder();

  private SparkMaxPIDController m_pidController;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;


  /** Creates a new CascadeSubsystem. */
  public Cascade() {
    m_pidController = motor1.getPIDController();

    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void cascadeDrive(double speed, double distance) {
    // var batteryVoltage = RobotController.getBatteryVoltage();
    // if (Math.max(Math.abs(Volt1), Math.abs(Volt2)) > batteryVoltage) {
    //   Volt1 *= batteryVoltage / 12.0;
    //   Volt2 *= batteryVoltage / 12.0;
    // }
    // motor1.setVoltage(Volt1);
    // motor2.setVoltage(Volt2);

    // m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
  }

  public double getEncoder1Count(){
    return m_encoder1.getPosition();
  }

  public double getEncoder2Count(){
    return m_encoder2.getPosition();
  }

  public double getEncoder1Velocity() {
    return m_encoder1.getVelocity();
  }

  public double getEncoder2Velocity() {
    return m_encoder2.getVelocity();
  }

  // public double getEncoder1DistanceInch() {
  //   return m_encoder1.getDistance();
  // }

  // public double getEncoder2DistanceInch() {
  //   return m_encoder2.getDistance();
  // }

  // public double getAverageDistanceInch() {
  //   return (m_encoder1.getDistance() + m_encoder2.getDistance()) / 2;
  // }

  public RelativeEncoder getEncoder1() {
    return m_encoder1;
  }

  public RelativeEncoder getEncoder2() {
    return m_encoder2;
  }
}
