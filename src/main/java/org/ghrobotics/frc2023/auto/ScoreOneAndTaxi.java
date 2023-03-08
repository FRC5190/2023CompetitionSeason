// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import java.util.ArrayList;

import org.ghrobotics.frc2023.Superstructure;
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

// Score One (High) and Taxi
// * line up in front of scoring column with space in between
// * align elevator, extender, and arm to scoring high position
// * score cube or cone
// * enter stow position
// * taxi backwards

public class ScoreOneAndTaxi extends SequentialCommandGroup {
    // Starting Positions (on blue side)
    private static final Pose2d kBotStartingPos = new Pose2d(1.9, 4.5, new Rotation2d(Math.PI));
    //private static final Pose2d kScoringPos = new Pose2d(2.5, 4.5, new Rotation2d(Math.PI));
    private static final Pose2d kEndPos = new Pose2d(4.5, 4.5, new Rotation2d());
    //private static final Pose2d kTopStartingPos = new Pose2d(1.900, 4.424, new Rotation2d());



  // Constructor
  public ScoreOneAndTaxi(Drivetrain drivetrain, Superstructure superstructure,
                        PoseEstimator pose_estimator, DriverStation.Alliance alliance) {
    // Check if we need to mirror poses
    boolean should_mirror = alliance == DriverStation.Alliance.Red;

    // Get starting positions
    //Pose2d kBotStartingPos = pose_estimator.getPosition();
    Pose2d start_pos = should_mirror ? mirror(kBotStartingPos) : kBotStartingPos;
    //Pose2d score_pos = should_mirror ? mirror(kScoringPos) : kScoringPos;
    Pose2d end_pos = should_mirror ? mirror(kEndPos) : kEndPos;


    //Trajectory t1 = TrajectoryGenerator.generateTrajectory(start_pos, new ArrayList<>(), score_pos, AutoConfig.kForwardConfig);
    Trajectory t1 = TrajectoryGenerator.generateTrajectory(start_pos, new ArrayList<>(), end_pos, AutoConfig.kReverseConfig);

    addCommands(
      new InstantCommand(() -> pose_estimator.resetPosition(start_pos)),

      superstructure.setPosition(Superstructure.Position.CUBE_L3),

      //new DriveTrajectory(drivetrain, pose_estimator, () -> t1),

      new ParallelCommandGroup(
        new WaitCommand(1.5),
        //Run Grabber
        superstructure.setGrabber(() -> 0.4, false).withTimeout(0.5)
      ),

      new ParallelCommandGroup(
        new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
        superstructure.setPosition(Superstructure.Position.STOW)
      ) /*
         new WaitCommand(1.5),
         superstructure.setPosition(Superstructure.Position.INTAKE),
         superstructure.setGrabber(() -> -0.4, true)*/
    );
  }

  //Flip Poses for Red Alliance
  private static Pose2d mirror(Pose2d pose) {
    return new Pose2d(16.54175 - pose.getX(), pose.getY(),
      new Rotation2d(Math.PI).minus(pose.getRotation()));
  }
}
