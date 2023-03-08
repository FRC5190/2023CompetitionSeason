// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;

public class AutoConfig {
  // Constraints
  public static final double kMaxVelocity = 3.0;
  public static final double kMaxAcceleration = 0.3;
  public static final double kMaxCentripetalAcceleration = 2.5;

  public static final double kBalanceRampApproachSpeed = 0.25;

  // Trajectory Configs
  public static final TrajectoryConfig kForwardConfig = new TrajectoryConfig(kMaxVelocity,
      kMaxAcceleration).addConstraint(
      new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration));

  public static final TrajectoryConfig kReverseConfig = new TrajectoryConfig(kMaxVelocity,
      kMaxAcceleration).setReversed(true).addConstraint(
      new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration));

  public static final TrajectoryConfig kReverseToBalanceConfig = new TrajectoryConfig(kMaxVelocity,
      kMaxAcceleration).setReversed(true).addConstraint(
       new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration)
      ).setEndVelocity(kBalanceRampApproachSpeed);

  // Flip Poses for Red Alliance
  public static Pose2d mirror(Pose2d pose) {
    return new Pose2d(16.54175 - pose.getX(), pose.getY(),
        new Rotation2d(Math.PI).minus(pose.getRotation()));
  }
}
