// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Cascade;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cascade;

public class CascadeControl extends CommandBase {
  private final Cascade m_cascade;
  private final double setpoint;
  /** Creates a new CascadeDistance. */
  public CascadeControl(double setpoint, Cascade cascade) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_cascade = cascade;
    this.setpoint = setpoint;
    addRequirements(cascade);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cascade.cascadeDrive(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_cascade.cascadeDrive(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_cascade.cascadeDrive(0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
