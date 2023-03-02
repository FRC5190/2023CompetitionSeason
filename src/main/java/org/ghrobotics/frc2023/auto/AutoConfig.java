// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;

public class AutoConfig {
  // Constraints
  public static final double kMaxVelocity = 0.5;
  public static final double kMaxAcceleration = 0.5;
  public static final double kMaxCentripetalAcceleration = 0.5;

  // Trajectory Configs
  public static final TrajectoryConfig kForwardConfig = new TrajectoryConfig(kMaxVelocity,
      kMaxAcceleration).addConstraint(
      new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration));

  public static final TrajectoryConfig kReverseConfig = new TrajectoryConfig(kMaxVelocity,
      kMaxAcceleration).setReversed(true).addConstraint(
      new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration));
}
