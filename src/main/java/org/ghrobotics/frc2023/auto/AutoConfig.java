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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public static final double kMaxVelocity = 0.5;
  public static final double kMaxAcceleration = 0.5;
  public static final double kMaxCentripetalAcceleration = 0.5;

  public static final TrajectoryConfig kForwardConfig =
      new TrajectoryConfig(kMaxVelocity, kMaxAcceleration)
          .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration));

  public static final TrajectoryConfig kReverseConfig =
      new TrajectoryConfig(kMaxVelocity, kMaxAcceleration)
          .setReversed(true)
          .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration));

  public Pose2d finalPos = Arena.tagPositions[0].transformBy(Arena.blueTransform[0]).toPose2d();
    public Trajectory kStartToLeftOfBlueTag = TrajectoryGenerator.generateTrajectory
        (pose_estimator_.getCurrentPose(), List.of(), finalPos, 
        kForwardConfig);

public static final Trajectory kStartToLeftOfBlueTagNV = TrajectoryGenerator.generateTrajectory
    (new Pose2d(0.522, 0.345, Rotation2d.fromDegrees(271.5)), List.of(),
    new Pose2d(0.422, 1.000, Rotation2d.fromDegrees(270)),
    kForwardConfig);

    //SmartDashboard.putNumber("End Pos X", finalPos.getX());
    //SmartDashboard.putNumber("End Pos Y", finalPose.getY());

}
