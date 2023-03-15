// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private boolean isIntakeUp;
  /** Creates a new Intake. */
  public Intake()  {  
    
  }

  @Override
  public void periodic() {
    System.out.println(IntakeConstants.CONVEYOR_SPEED);
    // This method will be called once per scheduler run
  }

  
  public void pullIn() {
    intakeMotor.set(IntakeConstants.INTAKE_SPEED);
  }

  public void Eject() {
    intakeMotor.set(-IntakeConstants.INTAKE_SPEED);
  }
  public void stopMotors() {
    intakeMotor.set(0);
  }
  public boolean isIntakeUp() {
    return isIntakeUp;
  }
}
