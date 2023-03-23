// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.ArrayList;
import org.ghrobotics.frc2023.Superstructure;
import org.ghrobotics.frc2023.commands.DriveBalance;
import org.ghrobotics.frc2023.commands.DriveTrajectory;
import org.ghrobotics.frc2023.commands.TurnToAngle;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

// Score High Cube Backwards
// * go on top of charge station and exit community
// * pick up cone/cube
// * move backwards
// * run balance command

public class ScoreCoopertitionExitBalanceBackwards extends SequentialCommandGroup {
  // Start location (middle)
  private static final Pose2d kStartPos = new Pose2d(1.9, 3.32, Rotation2d.fromDegrees(180));

  // Charge station location
  private static final Pose2d kChargeStation = new Pose2d(3.4, 3.32, Rotation2d.fromDegrees(180));


  /**
   * Creates a new ScoreCoopertitionExitBalanceBackwards.
   */
  public ScoreCoopertitionExitBalanceBackwards(Drivetrain drivetrain, Superstructure superstructure,
                                               PoseEstimator pose_estimator,
                                               DriverStation.Alliance alliance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    boolean should_mirror = alliance == DriverStation.Alliance.Red;
    Pose2d start_pos = should_mirror ? mirror(kStartPos) : kStartPos;
    Pose2d charge_station_pos = should_mirror ? mirror(kChargeStation) : kChargeStation;

    double angle = alliance == DriverStation.Alliance.Red ? 180 : 0;

    // Generate Trajectories
    Trajectory t1 = TrajectoryGenerator.generateTrajectory(
        start_pos,
        new ArrayList<>(),
        charge_station_pos,
        AutoConfig.kReversedRampConfig
    );

    addCommands(
        // Reset pose estimator to starting position
        new InstantCommand(() -> pose_estimator.resetPosition(start_pos)),

        // Cube L3
        new WaitCommand(0.3),
        superstructure.setPosition(Superstructure.Position.CONE_L2),
        superstructure.setGrabber(() -> 0.0, true).withTimeout(0.3),

        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
                new DriveBalance(drivetrain)
            ),
            superstructure.setPosition(Superstructure.Position.STOW).withTimeout(0.5)
        )
    );
  }

  //Flip Poses for Red Alliance
  private static Pose2d mirror(Pose2d pose) {
    return new Pose2d(16.54175 - pose.getX(), pose.getY(),
        new Rotation2d(Math.PI).minus(pose.getRotation()));
  }
}