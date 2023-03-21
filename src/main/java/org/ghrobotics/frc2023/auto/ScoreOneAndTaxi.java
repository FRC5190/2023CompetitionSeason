// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import java.util.ArrayList;
import java.util.List;

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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
    private static final Pose2d kTopStartingPos = new Pose2d(1.9, 4.5, Rotation2d.fromDegrees(0));
    private static final Pose2d kBotStartingPos = new Pose2d(1.9, 1.071, Rotation2d.fromDegrees(0));
    //private static final Pose2d kTurnPos = new Pose2d(4.5, 4.5, Rotation2d.fromDegrees(180));
    //private static final Pose2d kScoringPos = new Pose2d(1.9, 4.5, new Rotation2d(Math.PI));
    private static final Pose2d kTopEndPos = new Pose2d(6.46, 4.6, Rotation2d.fromDegrees(0));
    private static final Pose2d kBotEndPos = new Pose2d(6.46, 0.922,Rotation2d.fromDegrees(0));

  // Constructor
  public ScoreOneAndTaxi(Drivetrain drivetrain, Superstructure superstructure,
                        PoseEstimator pose_estimator, DriverStation.Alliance alliance,
                        AutoSelector.Grid grid_selection) {

      //Assign top or bottom position
      Pose2d start_pos = grid_selection == AutoSelector.Grid.TOP ? kTopStartingPos : kBotStartingPos;
      Pose2d cube_pos = grid_selection == AutoSelector.Grid.TOP ? kTopEndPos : kBotEndPos;
    
    // Check if we need to mirror poses
    boolean should_mirror = alliance == DriverStation.Alliance.Red;

    // Get starting positions
    //Pose2d kBotStartingPos = pose_estimator.getPosition();
    Pose2d start_pos_ = should_mirror ? mirror(start_pos) : start_pos;
    Pose2d cube_pos_ = should_mirror ? mirror(cube_pos) : cube_pos;


    //Trajectory t1 = TrajectoryGenerator.generateTrajectory(start_pos, new ArrayList<>(), score_pos, AutoConfig.kForwardConfig);
    Trajectory t1 = TrajectoryGenerator.generateTrajectory(start_pos_, new ArrayList<>(), cube_pos_, AutoConfig.kForwardConfig);
   // Trajectory t2 = TrajectoryGenerator.generateTrajectory(turn_pos, new ArrayList<>(), end_pos, AutoConfig.kForwardConfig);
   //Trajectory t2 = TrajectoryGenerator.generateTrajectory(end_pos, new ArrayList<>(), start_pos, AutoConfig.kReverseConfig);

    addCommands(
      new InstantCommand(() -> pose_estimator.resetPosition(start_pos_)),

      superstructure.setPosition(Superstructure.Position.BACK_EXHAUST).withTimeout(2),
      superstructure.setGrabber(() -> 0.4, false).withTimeout(0.5),
      superstructure.setPosition(Superstructure.Position.INTAKE),

      new ParallelCommandGroup( 
          new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
          superstructure.setGrabber(() -> -0.4, true).withTimeout(3.5)), 
      
      superstructure.setPosition(Superstructure.Position.STOW)

        //new WaitCommand(1.5),
        //Run Grabber
        
    //  ),
/*
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
          superstructure.setPosition(Superstructure.Position.INTAKE),
          superstructure.setGrabber(() -> -0.4, true).withTimeout(0.5))*/
     // ), 
     // new InstantCommand(() -> drivetrain.setVelocity(-1, 1), drivetrain),
     // new DriveTrajectory(drivetrain, pose_estimator, () -> t2)
      /*
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
