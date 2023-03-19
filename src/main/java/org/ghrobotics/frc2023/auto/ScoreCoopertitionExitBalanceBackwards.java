// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import org.ghrobotics.frc2023.Superstructure;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.ArrayList;
import org.ghrobotics.frc2023.commands.DriveBalance;
import org.ghrobotics.frc2023.commands.DriveTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Score High Cube Backwards
// * go on top of charge station and exit community
// * pick up cone/cube
// * move backwards
// * run balance command

public class ScoreCoopertitionExitBalanceBackwards extends SequentialCommandGroup {
  // Start location (middle)
  private static final Pose2d kStartPos = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(0));
  // Pickup location waypoint
  private static final Pose2d kPickupWaypoint = new Pose2d(5.75, 3.0, Rotation2d.fromDegrees(0));
  // Pickup location
  private static final Pose2d kPickupPos = new Pose2d(6.54, 2.30, Rotation2d.fromDegrees(0));
  // Charge station location
  private static final Pose2d kChargeStation = new Pose2d(5.0, 2.45, Rotation2d.fromDegrees(0));


  /** Creates a new ScoreCoopertitionExitBalanceBackwards. */
  public ScoreCoopertitionExitBalanceBackwards(Drivetrain drivetrain, Superstructure superstructure,
  PoseEstimator pose_estimator, DriverStation.Alliance alliance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    boolean should_mirror = alliance == DriverStation.Alliance.Red;
    Pose2d start_pos = should_mirror ? mirror(kStartPos) : kStartPos;
    Pose2d pickup_pos = should_mirror ? mirror(kPickupPos) : kPickupPos;
    Pose2d pickup_waypoint = should_mirror ? mirror(kPickupWaypoint) : kPickupWaypoint;
    Pose2d charge_station_pos = should_mirror ? mirror(kChargeStation) : kChargeStation;

    // Generate Trajectories
    Trajectory t1 = TrajectoryGenerator.generateTrajectory(
      start_pos,
      new ArrayList<>(),
      pickup_waypoint,
      AutoConfig.kForwardConfig
    );
    Trajectory t2 = TrajectoryGenerator.generateTrajectory(
      pickup_waypoint,
      new ArrayList<>(),
      pickup_pos,
      AutoConfig.kForwardConfig
    );
    Trajectory t3 = TrajectoryGenerator.generateTrajectory(
      pickup_pos,
      new ArrayList<>(),
      charge_station_pos,
      AutoConfig.kReverseConfig
    );


    addCommands(
        // Reset pose estimator to starting position
        new InstantCommand(() -> pose_estimator.resetPosition(start_pos)),

        // Exhaust cube from the back
        superstructure.setPosition(Superstructure.Position.BACK_EXHAUST).withTimeout(2),
        superstructure.setGrabber(() -> 0.67, true).withTimeout(0.5),

        // Drive to pickup cube
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
                new DriveTrajectory(drivetrain, pose_estimator, () -> t2)
            ),
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                superstructure.setPosition(Superstructure.Position.INTAKE),
                superstructure.setGrabber(() -> 0.67, true).withTimeout(1.5))
        ),

        
        // Drive to charge station
        new ParallelCommandGroup(
            new DriveTrajectory(drivetrain, pose_estimator, () -> t3),
            superstructure.setPosition(Superstructure.Position.STOW)
        ),

        // Drive backwards to balance
        new DriveBalance(drivetrain)
        
    );
  }

  //Flip Poses for Red Alliance
  private static Pose2d mirror(Pose2d pose) {
    return new Pose2d(16.54175 - pose.getX(), pose.getY(),
      new Rotation2d(Math.PI).minus(pose.getRotation()));
  }
}
