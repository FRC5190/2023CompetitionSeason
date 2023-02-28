// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

/** Add your docs here. */
public class DriveTrajectory extends CommandBase{
    private final Drivetrain drivetrain_;
    private final PoseEstimator pose_estimator;

    // Objects needed to track trajectory.
    private final Trajectory trajectory_;
    private final Timer timer_;

    public DriveTrajectory(Drivetrain drivetrain, PoseEstimator poseEstimator, Trajectory trajectory) {

        drivetrain_ = drivetrain;
        pose_estimator = poseEstimator;
        trajectory_ = trajectory;

        timer_ = new Timer();

        addRequirements(drivetrain_);
    }

    @Override
    public void initialize() {
      // Start the timer.
      timer_.start();
    }
  
    @Override
    public void execute() {
      // Get time elapsed.
      double t = timer_.get();
  
      // Get desired state at current time.
      Trajectory.State desired_state = trajectory_.sample(t);
  
      // Get robot pose at the current time.
      Pose2d current_state = pose_estimator.getCurrentPose();
  
      // Calculate chassis speeds to track desired state.
      ChassisSpeeds wanted_chassis_speeds = drivetrain_.getRamseteController().calculate(
          current_state, desired_state);
  
      // Convert to wheel speeds.
      DifferentialDriveWheelSpeeds wanted_wheel_speeds = drivetrain_.getKinematics().toWheelSpeeds(
          wanted_chassis_speeds);

          SmartDashboard.putNumber("Wanted Wheel Speeds Left", wanted_wheel_speeds.leftMetersPerSecond);
          SmartDashboard.putNumber("Wanted Wheel Speeds Right", wanted_wheel_speeds.rightMetersPerSecond);
  
      // Set wheel speeds on drivetrain.
     /* drivetrain_.setVelocity(wanted_wheel_speeds.leftMetersPerSecond,
          wanted_wheel_speeds.rightMetersPerSecond);*/
    }
  
    @Override
    public void end(boolean interrupted) {
      // If we are interrupted, something went wrong; set speeds to 0.
      if (interrupted)
        drivetrain_.setPercent(0, 0);
  
      // Stop and reset the timer.
      timer_.stop();
      timer_.reset();
  
      // Debug: print error
      Pose2d end_pose = trajectory_.sample(trajectory_.getTotalTimeSeconds()).poseMeters;
      Pose2d robot_pose = pose_estimator.getCurrentPose();
  
      System.out.printf("X Error (in): %3.2f, Y Error (in): %3.2f, Theta Error (deg): %3.2f\n",
          Units.metersToFeet(end_pose.getX() - robot_pose.getX()),
          Units.metersToFeet(end_pose.getY() - robot_pose.getY()),
          end_pose.getRotation().getDegrees() - robot_pose.getRotation().getDegrees());
    }
  
    @Override
    public boolean isFinished() {
      // We are done when the elapsed time is greater than the trajectory's total time.
      return timer_.get() > trajectory_.getTotalTimeSeconds();
    }

}
