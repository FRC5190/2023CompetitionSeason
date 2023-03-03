// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase {
  // Subsystems
  private final Drivetrain drivetrain_;
  private final Limelight limelight_;

  // Alive Filter for Limelight
  private final LinearFilter alive_filter_;
  private boolean is_alive_ = false;

  // Pose Estimator Confidence Values
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05,
      Units.degreesToRadians(5));
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5,
      Units.degreesToRadians(10));

  // Pose Estimator
  private final DifferentialDrivePoseEstimator pose_estimator_;

  // Reset Position From Vision
  private boolean reset_position_from_vision_ = false;

  // Primary Tag ID
  private int primary_tag_id_ = -1;

  // Constructor
  public PoseEstimator(Drivetrain drivetrain, Limelight limelight) {
    // Assign member variables
    drivetrain_ = drivetrain;
    limelight_ = limelight;

    // Initialize alive filter
    alive_filter_ = LinearFilter.movingAverage(10);

    // Initialize pose estimator
    pose_estimator_ = new DifferentialDrivePoseEstimator(drivetrain_.getKinematics(),
        new Rotation2d(drivetrain_.getAngle()), drivetrain_.getLeftPosition(),
        drivetrain_.getRightPosition(),
        new Pose2d(), stateStdDevs, visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    // Get limelight processing and capture latency
    double processing_latency = limelight_.getProcessingLatency();
    double capture_latency = limelight_.getCaptureLatency();

    // Update alive filter
    is_alive_ = alive_filter_.calculate(processing_latency) > 11;

    // Update the pose estimator with new measurements if we have a new target.
    if (limelight_.hasTarget()) {
      // Calculate timestamp of capture
      double timestamp =
          Timer.getFPGATimestamp() - (processing_latency / 1000) - (capture_latency / 1000);

      // Update primary tag id
      primary_tag_id_ = limelight_.getID();

      // Use the robot pose with blue alliance origin
      double[] raw_pose = limelight_.getBlueBotPose();

      // Extract 3d pose from double[]
      Pose3d robot_pose = new Pose3d(raw_pose[0], raw_pose[1], raw_pose[2],
          new Rotation3d(raw_pose[3], raw_pose[4], raw_pose[5]));

      // Check whether we want to reset position from vision
      if (reset_position_from_vision_) {
        resetPosition(robot_pose.toPose2d());
        reset_position_from_vision_ = false;
      } else {
        // Add vision measurement to estimator
        pose_estimator_.addVisionMeasurement(robot_pose.toPose2d(), timestamp);
      }
    }

    // Add encoders / gyro measurement
    pose_estimator_.update(new Rotation2d(drivetrain_.getAngle()), drivetrain_.getLeftPosition(),
        drivetrain_.getRightPosition());
  }

  // Vision Alive Getter
  public boolean isVisionAlive() {
    return is_alive_;
  }

  // Primary Tag ID Getter
  public int getVisionPrimaryTagId() {
    return primary_tag_id_;
  }

  // Position Getter
  public Pose2d getPosition() {
    return pose_estimator_.getEstimatedPosition();
  }

  // Reset Position
  public void resetPosition(Pose2d pose) {
    pose_estimator_.resetPosition(new Rotation2d(drivetrain_.getAngle()),
        drivetrain_.getLeftPosition(),
        drivetrain_.getRightPosition(), pose);
  }

  // Reset Position from Vision
  public void resetPositionFromVision() {
    reset_position_from_vision_ = true;
  }
}
