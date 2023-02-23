// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;
import java.util.List;
import org.ghrobotics.frc2023.subsystems.Gyroscope;
import org.ghrobotics.frc2023.commands.DriveTeleop;
import org.ghrobotics.frc2023.Telemetry;
import org.ghrobotics.frc2023.Limelight;
import org.ghrobotics.frc2023.Arena;

/** Add your docs here. */
public class AutoConfig {

    private final Drivetrain drivetrain_ = new Drivetrain();
    private final Limelight limelight_ = new Limelight("limelight");
    private final Gyroscope gyro_ = new Gyroscope();        
    private final PoseEstimator pose_estimator_ = new PoseEstimator(limelight_, drivetrain_, gyro_);
   
    // Constraints
  public static final double kMaxVelocity = 3.0;
  public static final double kMaxAcceleration = 1.8;
  public static final double kMaxCentripetalAcceleration = 1.5;

  public static final TrajectoryConfig kForwardConfig =
      new TrajectoryConfig(kMaxVelocity, kMaxAcceleration)
          .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration));

  public static final TrajectoryConfig kReverseConfig =
      new TrajectoryConfig(kMaxVelocity, kMaxAcceleration)
          .setReversed(true)
          .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration));

  public final Trajectory kStartToLeftOfBlueTag = TrajectoryGenerator.generateTrajectory
    (pose_estimator_.getCurrentPose(), List.of(), Arena.tagPositions[0].transformBy(Arena.blueTransform[0][0]).toPose2d(), 
    kForwardConfig);


}
