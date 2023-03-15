// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import java.util.ArrayList;

import org.ghrobotics.frc2023.Superstructure;
import org.ghrobotics.frc2023.commands.DriveBalance;
import org.ghrobotics.frc2023.commands.DriveTrajectory;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Score forward cube L3 
// * enter the charge station backwards
// * run the drive balance command

public class ScoreForwardBalanceBackward extends SequentialCommandGroup {
  private static final Pose2d kStartPos = new Pose2d(1.750, 2.749, Rotation2d.fromDegrees(180));
  private static final Pose2d kOnChargeStation = new Pose2d(3.50, 2.755, Rotation2d.fromDegrees(180));

  /** Creates a new ScoreForwardBalanceBackward. */
  public ScoreForwardBalanceBackward(Drivetrain drivetrain, Superstructure superstructure,
    PoseEstimator pose_estimator, DriverStation.Alliance alliance){
    //AutoSelector.Grid grid_selection) {

      boolean should_mirror = alliance == DriverStation.Alliance.Red;

      Pose2d start_pos  = should_mirror ? mirror(kStartPos) : kStartPos;
      Pose2d charge_station_pos = should_mirror ? mirror(kOnChargeStation) : kOnChargeStation;

      Trajectory t1 = TrajectoryGenerator.generateTrajectory(
        start_pos, new ArrayList<>(), charge_station_pos,
        AutoConfig.kReverseConfig);


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Reset pose estimator to starting position
      new InstantCommand(() -> pose_estimator.resetPosition(start_pos)),

      new WaitCommand(1.0),
/*
      new ParallelCommandGroup(
        superstructure.setPosition(Superstructure.Position.CUBE_L3).withTimeout(6.5),
          new SequentialCommandGroup(
            new WaitCommand(3.0),
            superstructure.setGrabber(() -> 0.67, true).withTimeout(2.5)
          )
      ), */

      new ParallelCommandGroup(
        new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
        superstructure.setPosition(Superstructure.Position.STOW)
      ),

      new DriveBalance(drivetrain)
    );
  }

  //Flip Poses for Red Alliance
  private static Pose2d mirror(Pose2d pose) {
    return new Pose2d(16.54175 - pose.getX(), pose.getY(),
      new Rotation2d(Math.PI).minus(pose.getRotation()));
  }

}
