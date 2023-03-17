// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class ForwardPID extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public ForwardPID(double setPoint, double targetAngleDegrees, Drivetrain drive) {
    super(
        new ProfiledPIDController(
            DriveConstants.kP,
            DriveConstants.kI,
            DriveConstants.kD,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxTurnRateDegPerS,
                DriveConstants.kMaxTurnAccelerationDegPerSSquared)),
        // Close loop on heading
        drive::currentPosition,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        (output, setpoint) -> drive.arcadeDrive(output, 0),
        // Require the drive
        drive); //double goal, BiConsumer<Double, State> useOutput, Subsystem... requirement
  }

  @Override
  public void execute() {
      // TODO Auto-generated method stub
          // Set the controller to be continuous (because it is an angle controller)
      super.getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
      super.getController().setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
