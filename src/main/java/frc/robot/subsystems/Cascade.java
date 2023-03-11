// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CascadeConstants;

public class Cascade extends SubsystemBase {

  public static CANSparkMax motor1 = new CANSparkMax(CascadeConstants.kCascadeMotor1Port, MotorType.kBrushless);
  public static CANSparkMax motor2 = new CANSparkMax(CascadeConstants.kCascadeMotor1Port, MotorType.kBrushless);

  MotorControllerGroup m_cascadeMotors = new MotorControllerGroup(motor1, motor2);
  
  private final Encoder m_encoder1 = 
      new Encoder(
        CascadeConstants.kCascadeEncoder1Ports[0],
        CascadeConstants.kCascadeEncoder1Ports[1],
        CascadeConstants.kCascadeEncoder1Reversed);
  
  private final Encoder m_encoder2 = 
      new Encoder(
        CascadeConstants.kCascadeEncoder2Ports[0],
        CascadeConstants.kCascadeEncoder2Ports[1],
        CascadeConstants.kCascadeEncoder2Reversed);


  /** Creates a new CascadeSubsystem. */
  public Cascade() {
    m_encoder1.setDistancePerPulse(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void cascadeDriveVolts(double Volt1, double Volt2) {
    var batteryVoltage = RobotController.getBatteryVoltage();
    if (Math.max(Math.abs(Volt1), Math.abs(Volt2)) > batteryVoltage) {
      Volt1 *= batteryVoltage / 12.0;
      Volt2 *= batteryVoltage / 12.0;
    }
    motor1.setVoltage(Volt1);
    motor2.setVoltage(Volt2);
  }

  public int getEncoder1Count(){
    return m_encoder1.get();
  }

  public int getEncoder2Count(){
    return m_encoder2.get();
  }

  public double getEncoder1Velocity() {
    return m_encoder1.getRate();
  }

  public double getEncoder2Velocity() {
    return m_encoder2.getRate();
  }

  public double getEncoder1DistanceInch() {
    return m_encoder1.getDistance();
  }

  public double getEncoder2DistanceInch() {
    return m_encoder2.getDistance();
  }

  public double getAverageDistanceInch() {
    return (m_encoder1.getDistance() + m_encoder2.getDistance()) / 2;
  }

  public Encoder getEncoder1() {
    return m_encoder1;
  }

  public Encoder getEncoder2() {
    return m_encoder2;
  }

  public void resetEncoders() {
    m_encoder1.reset();
    m_encoder2.reset();
  }
}
