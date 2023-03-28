// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.ArrayList;
import org.ghrobotics.frc2023.Superstructure;
import org.ghrobotics.frc2023.commands.DriveBalance;
import org.ghrobotics.frc2023.commands.DriveTrajectory;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

public class ScoreCoopertitionBalanceBackwards extends SequentialCommandGroup {
  // Poses (assuming origin on blue side)
  private static final Pose2d kStartPos = new Pose2d(1.9, 3.32, Rotation2d.fromDegrees(180));
  private static final Pose2d kChargeStation = new Pose2d(3.5, 3.32, Rotation2d.fromDegrees(180));

  // Constructor
  public ScoreCoopertitionBalanceBackwards(Drivetrain drivetrain, Superstructure superstructure,
                                           PoseEstimator pose_estimator,
                                           DriverStation.Alliance alliance) {
    // Get poses
    Pose2d start_pos = AutoConfig.adjustPoseForAlliance(kStartPos, alliance);
    Pose2d charge_station_pos = AutoConfig.adjustPoseForAlliance(kChargeStation, alliance);

    // Generate trajectory
    Trajectory t1 = TrajectoryGenerator.generateTrajectory(
        start_pos, new ArrayList<>(), charge_station_pos, AutoConfig.kReverseConfig);

    // Create command group
    addCommands(
        // Reset pose estimator to starting position
        new InstantCommand(() -> pose_estimator.resetPosition(start_pos)),

        // Cube L3
        new WaitCommand(0.3),
        superstructure.setPosition(Superstructure.Position.CONE_L2),
        superstructure.setGrabber(() -> 0.0, true).withTimeout(0.3),

        // Balance
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
                new DriveBalance(drivetrain)
            ),
            superstructure.setPosition(Superstructure.Position.STOW).withTimeout(0.5)
        )
    );
  }
}