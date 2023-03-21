// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.commands;

import org.ghrobotics.frc2023.auto.AutoConfig;
import org.ghrobotics.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToDegreesProfiled extends ProfiledPIDCommand {
  Drivetrain drive_;
  double targetangleradians_;
  /** Creates a new TurnToDegreesProfiled. */
  public TurnToDegreesProfiled(double targetAngleRadians, Drivetrain drive) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            1.0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(AutoConfig.kMaxVelocity, AutoConfig.kMaxAcceleration)),
        // This should return the measurement
        () -> drive.getAngle(),
            // This should return the goal (can also be a constant)
        () -> targetAngleRadians,
        // This uses the output
        (output, setpoint) -> drive.setPercent(-output, output), 
          // Use the output (and setpoint, if desired) here
        drive);
  drive_ = drive;
  targetangleradians_ = targetAngleRadians;
  //getController().setGoal(targetAngleDegrees);
// Set the controller to be continuous (because it is an angle controller)
  getController().enableContinuousInput(0, 360);
// Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
// setpoint before it is considered as having reached the reference
  getController().setTolerance(Math.toRadians(2), 3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
    //System.out.println("drive.getAngle() == targetangledegrees_" + Math.toDegrees(drive_.getAngle()) + " == " + targetangleradians_);
    //return (drive_.getAngle()/targetangleradians_) > .98;
  }
}
