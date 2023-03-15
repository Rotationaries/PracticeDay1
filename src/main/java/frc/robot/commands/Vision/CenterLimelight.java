// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Drivetrain;

public class CenterLimelight extends CommandBase {
  private Drivetrain drive;
  private double xPos = drive.getPose().getRotation().getDegrees();

  public CenterLimelight(Drivetrain drive) {
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // CommandScheduler.getInstance().schedule(new TurnDegrees(LimelightConstants.speed, LimelightConstants.tx, drive));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xPos-drive.getPose().getRotation().getDegrees() == LimelightConstants.tx;
  }
}
