// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Cascade;

public class CascadePID extends CommandBase {
  Cascade cascade; 
  double kP = 1, kI = 1, kD = 1;
  double setpoint;
  double velocity;
  PIDController pid = new PIDController(kP, kI, kD);
  double tracker;
  
  /** Creates a new CascadePID. */
  public CascadePID(double kP, double kI, double kD, double setpoint, double velocity) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.setpoint = setpoint;
    this.velocity = velocity;
    tracker = cascade.getAverageDistanceInch();

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.setTolerance(2, 10);
    pid.setSetpoint(setpoint);
    while(!pid.atSetpoint()) {
      tracker = cascade.getAverageDistanceInch();
      setpoint -= cascade.getEncoder1Count();
      CommandScheduler.getInstance().schedule(new CascadeDistance(velocity, pid.calculate(cascade.getAverageDistanceInch(), setpoint), cascade));
    }
    MathUtil.clamp(pid.calculate(cascade.getAverageDistanceInch(), setpoint), -0.5, 0.5);
    //motor.set(pid.calculate(encoder.getDistance(), setpoint));    
  }

  public PIDController getPID() {
    return pid;
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
