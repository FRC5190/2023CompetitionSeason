// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

public class TurnToAngle extends ProfiledPIDCommand {
  // Constructor
  public TurnToAngle(double target_angle, Drivetrain drivetrain,
                     PoseEstimator pose_estimator) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.6,
            0,
            0.02,

            // The motion profile constraints
            new TrapezoidProfile.Constraints(20, 4)
        ),
        // This should return the measurement
        () -> pose_estimator.getPosition().getRotation().getRadians(),
        // This should return the goal (can also be a constant)
        target_angle,
        // This uses the output
        (output, setpoint) -> drivetrain.setPercent(-output, output),
        // Subsystem requirement
        drivetrain);

    // Set controller goal
    getController().setGoal(target_angle);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-Math.PI, Math.PI);

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(Math.toRadians(10));

  }

  @Override
  public boolean isFinished() {
    return m_controller.atGoal();
  }
}
