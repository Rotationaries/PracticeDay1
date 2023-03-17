// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CascadeConstants;

public class Cascade extends SubsystemBase {

  public static CANSparkMax motor1 = new CANSparkMax(CascadeConstants.kCascadeMotor1Port, MotorType.kBrushless);
  public static CANSparkMax motor2 = new CANSparkMax(CascadeConstants.kCascadeMotor1Port, MotorType.kBrushless);

  MotorControllerGroup m_cascadeMotors = new MotorControllerGroup(motor1, motor2);
  
  private final RelativeEncoder m_encoder1 = motor1.getEncoder();
  
  private final RelativeEncoder m_encoder2 = motor2.getEncoder();

  private SparkMaxPIDController m_pidController1, m_pidController2;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  
  public boolean atStage;

  private final Joystick joystick = new Joystick (0);


  /** Creates a new CascadeSubsystem. */
  public Cascade() {
    m_pidController1 = motor1.getPIDController();

    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    m_pidController1.setP(kP);
    m_pidController1.setI(kI);
    m_pidController1.setD(kD);
    m_pidController1.setIZone(kIz);
    m_pidController1.setFF(kFF);
    m_pidController1.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot1 = 0;
    m_pidController1.setSmartMotionMaxVelocity(maxVel, smartMotionSlot1);
    m_pidController1.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot1);
    m_pidController1.setSmartMotionMaxAccel(maxAcc, smartMotionSlot1);
    m_pidController1.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot1);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain 1", kP);
    SmartDashboard.putNumber("I Gain 1", kI);
    SmartDashboard.putNumber("D Gain 1", kD);
    SmartDashboard.putNumber("I Zone 1", kIz);
    SmartDashboard.putNumber("Feed Forward 1", kFF);
    SmartDashboard.putNumber("Max Output 1", kMaxOutput);
    SmartDashboard.putNumber("Min Output 1", kMinOutput);

    // display Smart Motion coefficientso
    SmartDashboard.putNumber("Max Velocity 1", maxVel);
    SmartDashboard.putNumber("Min Velocity 1", minVel);
    SmartDashboard.putNumber("Max Acceleration 1", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error 1", allowedErr);
    SmartDashboard.putNumber("Set Position 1", 0);
    SmartDashboard.putNumber("Set Velocity 1", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode 1", true);


    m_pidController2.setP(kP);
    m_pidController2.setI(kI);
    m_pidController2.setD(kD);
    m_pidController2.setIZone(kIz);
    m_pidController2.setFF(kFF);
    m_pidController2.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot2 = 0;
    m_pidController2.setSmartMotionMaxVelocity(maxVel, smartMotionSlot2);
    m_pidController2.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot2);
    m_pidController2.setSmartMotionMaxAccel(maxAcc, smartMotionSlot2);
    m_pidController2.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot2);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain 2", kP);
    SmartDashboard.putNumber("I Gain 2", kI);
    SmartDashboard.putNumber("D Gain 2", kD);
    SmartDashboard.putNumber("I Zone 2", kIz);
    SmartDashboard.putNumber("Feed Forward 2", kFF);
    SmartDashboard.putNumber("Max Output 2", kMaxOutput);
    SmartDashboard.putNumber("Min Output 2", kMinOutput);

    // display Smart Motion coefficientso
    SmartDashboard.putNumber("Max Velocity 2", maxVel);
    SmartDashboard.putNumber("Min Velocity 2", minVel);
    SmartDashboard.putNumber("Max Acceleration 2", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error 2", allowedErr);
    SmartDashboard.putNumber("Set Position 2", 0);
    SmartDashboard.putNumber("Set Velocity 2", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode 2", true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double p = SmartDashboard.getNumber("P Gain 1", 0);
    double i = SmartDashboard.getNumber("I Gain 1", 0);
    double d = SmartDashboard.getNumber("D Gain 1", 0);
    double iz = SmartDashboard.getNumber("I Zone 1", 0);
    double ff = SmartDashboard.getNumber("Feed Forward 1", 0);
    double max = SmartDashboard.getNumber("Max Output 1", 0);
    double min = SmartDashboard.getNumber("Min Output 1", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity 1", 0);
    double minV = SmartDashboard.getNumber("Min Velocity 1", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration 1", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error 1", 0);

    if((p != kP)) { m_pidController1.setP(p); kP = p; }
    if((i != kI)) { m_pidController1.setI(i); kI = i; }
    if((d != kD)) { m_pidController1.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController1.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController1.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController1.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_pidController1.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { m_pidController1.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { m_pidController1.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_pidController1.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    double processVariable;
    boolean mode = SmartDashboard.getBoolean("Mode", false);
    // setPoint = SmartDashboard.getNumber("Set Position", 0);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
    processVariable = m_encoder1.getPosition(); 
    
    // SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", motor1.getAppliedOutput());

    cascadeDrive();

  }

  public void cascadeDrive() {
    atStage = false;
    // var batteryVoltage = RobotController.getBatteryVoltage();
    // if (Math.max(Math.abs(Volt1), Math.abs(Volt2)) > batteryVoltage) {
    //   Volt1 *= batteryVoltage / 12.0;
    //   Volt2 *= batteryVoltage / 12.0;
    // }
    // motor1.setVoltage(Volt1);
    // motor2.setVoltage(Volt2);
    if(joystick.getRawButtonPressed(7)){
      m_pidController1.setReference(CascadeConstants.kstage0, CANSparkMax.ControlType.kSmartMotion);
      m_pidController2.setReference(CascadeConstants.kstage0, CANSparkMax.ControlType.kSmartMotion);
    }
    if(joystick.getRawButtonPressed(8)){
      m_pidController1.setReference(CascadeConstants.kstage1, CANSparkMax.ControlType.kSmartMotion);
      m_pidController2.setReference(CascadeConstants.kstage1, CANSparkMax.ControlType.kSmartMotion);
    }
    if(joystick.getRawButtonPressed(9)){
      m_pidController1.setReference(CascadeConstants.kstage2, CANSparkMax.ControlType.kSmartMotion);
      m_pidController2.setReference(CascadeConstants.kstage2, CANSparkMax.ControlType.kSmartMotion);
    }
    if(joystick.getRawButtonPressed(10)){
      m_pidController1.setReference(CascadeConstants.kstage3, CANSparkMax.ControlType.kSmartMotion);
      m_pidController2.setReference(CascadeConstants.kstage3, CANSparkMax.ControlType.kSmartMotion);
    }
    atStage = true;
    
  }

  public void cascadeDrive(double setpoint) {
    atStage = false;
    // var batteryVoltage = RobotController.getBatteryVoltage();
    // if (Math.max(Math.abs(Volt1), Math.abs(Volt2)) > batteryVoltage) {
    //   Volt1 *= batteryVoltage / 12.0;
    //   Volt2 *= batteryVoltage / 12.0;
    // }
    // motor1.setVoltage(Volt1);
    // motor2.setVoltage(Volt2);
    if(joystick.getRawButtonPressed(7)){
      m_pidController1.setReference(CascadeConstants.kstage0, CANSparkMax.ControlType.kSmartMotion);
      m_pidController2.setReference(CascadeConstants.kstage0, CANSparkMax.ControlType.kSmartMotion);
    }
    if(joystick.getRawButtonPressed(8)){
      m_pidController1.setReference(CascadeConstants.kstage1, CANSparkMax.ControlType.kSmartMotion);
      m_pidController2.setReference(CascadeConstants.kstage1, CANSparkMax.ControlType.kSmartMotion);
    }
    if(joystick.getRawButtonPressed(9)){
      m_pidController1.setReference(CascadeConstants.kstage2, CANSparkMax.ControlType.kSmartMotion);
      m_pidController2.setReference(CascadeConstants.kstage2, CANSparkMax.ControlType.kSmartMotion);
    }
    if(joystick.getRawButtonPressed(10)){
      m_pidController1.setReference(CascadeConstants.kstage3, CANSparkMax.ControlType.kSmartMotion);
      m_pidController2.setReference(CascadeConstants.kstage3, CANSparkMax.ControlType.kSmartMotion);
    }
    atStage = true;
    
  } 

  public boolean atBottom() {
    return m_encoder1.getPosition() == 0 && m_encoder2.getPosition() == 0;
  }

  public boolean atTop() {
    return motor1.getOutputCurrent() == CascadeConstants.stallCurrent && 
    motor2.getOutputCurrent() == CascadeConstants.stallCurrent && !atBottom();
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
