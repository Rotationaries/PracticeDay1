// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cascade;

public class CascadeDistance extends CommandBase {
  private final Cascade m_cascade;
  private final double m_distance;
  private final double m_speed;
  /** Creates a new CascadeDistance. */
  public CascadeDistance(double speed, double distance, Cascade cascade) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_distance = distance;
    m_speed = speed;
    m_cascade = cascade;
    addRequirements(cascade);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cascade.cascadeDriveVolts(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_cascade.cascadeDriveVolts()
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
