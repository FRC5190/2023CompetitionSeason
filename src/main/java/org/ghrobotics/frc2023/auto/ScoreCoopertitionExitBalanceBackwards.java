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
import org.ghrobotics.frc2023.commands.TurnToAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.lang.Math;

// Score High Cube Backwards
// * go on top of charge station and exit community
// * pick up cone/cube
// * move backwards
// * run balance command

public class ScoreCoopertitionExitBalanceBackwards extends SequentialCommandGroup {
  // Start location (middle)
  private static final Pose2d kStartPos = new Pose2d(1.9, 2.77, Rotation2d.fromDegrees(180));
  // Pickup location waypoint
  private static final Pose2d kPickupWaypoint = new Pose2d(5.5, 2.5, Rotation2d.fromDegrees(180));
  // Pickup location turn
  private static final Pose2d kPickupPosTurn = new Pose2d(5.5, 2.5, Rotation2d.fromDegrees(0));
  // Pickup location
  private static final Pose2d kPickupPos = new Pose2d(6.37, 2.20, Rotation2d.fromDegrees(0));
  // Charge station location
  private static final Pose2d kChargeStation = new Pose2d(4.26, 2.45, Rotation2d.fromDegrees(0));


  /** Creates a new ScoreCoopertitionExitBalanceBackwards. */
  public ScoreCoopertitionExitBalanceBackwards(Drivetrain drivetrain, Superstructure superstructure,
  PoseEstimator pose_estimator, DriverStation.Alliance alliance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    boolean should_mirror = alliance == DriverStation.Alliance.Red;
    Pose2d start_pos = should_mirror ? mirror(kStartPos) : kStartPos;
    Pose2d pickup_pos = should_mirror ? mirror(kPickupPos) : kPickupPos;
    Pose2d pickup_waypoint = should_mirror ? mirror(kPickupWaypoint) : kPickupWaypoint;
    Pose2d pickup_pos_turn = should_mirror ? mirror(kPickupPosTurn) : kPickupPosTurn;
    Pose2d charge_station_pos = should_mirror ? mirror(kChargeStation) : kChargeStation;

    // Generate Trajectories
    Trajectory t1 = TrajectoryGenerator.generateTrajectory(
      start_pos,
      new ArrayList<>(),
      pickup_waypoint,
      AutoConfig.kReverseConfig
    );
    Trajectory t2 = TrajectoryGenerator.generateTrajectory(
      pickup_pos_turn,
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

        // Cube L3
        new ParallelCommandGroup(
          superstructure.setPosition(Superstructure.Position.CUBE_L3).withTimeout(6.5),
            new SequentialCommandGroup(
              new WaitCommand(3.0),
              superstructure.setGrabber(() -> 0, true).withTimeout(0.5)
            )
        ), 
        superstructure.setPosition(Superstructure.Position.STOW).withTimeout(0.5),
        // superstructure.setPosition(Superstructure.Position.STOW).withTimeout(0.5),
        // Drive to pickup cube
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
                new TurnToAngle(Math.toRadians(-165), drivetrain, pose_estimator),
                new DriveTrajectory(drivetrain, pose_estimator, () -> t2)
            ),
            new SequentialCommandGroup(
                new WaitCommand(2.5),
                superstructure.setPosition(Superstructure.Position.INTAKE),
                superstructure.setGrabber(() -> 0, true).withTimeout(1.5))
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