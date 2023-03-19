// i am Deceased

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import java.util.ArrayList;
import java.util.List;

import org.ghrobotics.frc2023.Superstructure;
import org.ghrobotics.frc2023.commands.DriveTrajectory;
import org.ghrobotics.frc2023.commands.TurnDegrees;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//Score cube (high) backwards
// * pickup another cube outside community moving forward
// * enter community and score second cube mid
// * exit community again and go to the previous cube way point
// * turn to the side and pick up third game piece

public class ScoreTwoAndTaxi extends SequentialCommandGroup {
  // Positions (on blue side)
  //Top Routine
  private static final Pose2d kTopStartingPos = new Pose2d(1.9, 4.5, Rotation2d.fromDegrees(0));
  private static final Pose2d kTopFirstPickupPos = new Pose2d(6.46, 4.6, Rotation2d.fromDegrees(0));
  private static final Pose2d kTopSecondScorePos = new Pose2d(2.5, 4.6, Rotation2d.fromDegrees(180));
  // private static final Pose2d kTopSecondScorePos = new Pose2d(2.5, 4.6, new Rotation2d(Math.PI));
  private static final Pose2d kTopPickupWaypoint = new Pose2d(4.95, 4.6, Rotation2d.fromDegrees(0));
  private static final Pose2d kTopEndPos = new Pose2d(6.95, 4.022, Rotation2d.fromDegrees(-75 + 360));
  // private static final Pose2d kTopEndPos = new Pose2d(6.95, 4.022, new Rotation2d(-75 * Math.PI / 180));

  //Bottom Routine 
  private static final Pose2d kBotStartingPos = new Pose2d(1.9, 1.071, Rotation2d.fromDegrees(0));
  private static final Pose2d kBotFirstPickupPos = new Pose2d(6.46, 0.922,Rotation2d.fromDegrees(0));
  private static final Pose2d kBotSecondScorePos = new Pose2d(2.5, 0.922, Rotation2d.fromDegrees(5));
  private static final Pose2d kBotPickupWaypoint = new Pose2d(6.55, 0.922,Rotation2d.fromDegrees(0)); //change rot2d
  private static final Pose2d kBotEndPos = new Pose2d(6.95, 1.5, Rotation2d.fromDegrees(75)); //change points6.9, 75

  //Constructor
  public ScoreTwoAndTaxi(Drivetrain drivetrain, Superstructure superstructure,
  PoseEstimator pose_estimator, DriverStation.Alliance alliance,
  AutoSelector.Grid grid_selection) {

    //Assign top or bottom position
    Pose2d start_pos = grid_selection == AutoSelector.Grid.TOP ? kTopStartingPos : kBotStartingPos;
    Pose2d cube_pos = grid_selection == AutoSelector.Grid.TOP ? kTopFirstPickupPos : kBotFirstPickupPos;
    Pose2d second_score_pos = grid_selection == AutoSelector.Grid.TOP ? kTopSecondScorePos : kBotSecondScorePos;
    Pose2d interior_waypoint = grid_selection == AutoSelector.Grid.TOP ? kTopPickupWaypoint : kBotPickupWaypoint;
    Pose2d end_pos = grid_selection == AutoSelector.Grid.TOP ? kTopEndPos : kBotEndPos;

    // Check if we need to mirror poses
    boolean should_mirror = alliance == DriverStation.Alliance.Red;

    Pose2d start_pos_ = should_mirror ? mirror(start_pos) : start_pos;
    Pose2d cube_pos_ = should_mirror ? mirror(cube_pos) : cube_pos;
    Pose2d second_score_pos_ = should_mirror ? mirror(second_score_pos) : second_score_pos;
    Pose2d interior_waypoint_ = should_mirror ? mirror(interior_waypoint) : interior_waypoint;
    Pose2d end_pos_ = should_mirror ? mirror(end_pos) : end_pos;

    Trajectory t1 = TrajectoryGenerator.generateTrajectory(start_pos_, new ArrayList<>(), cube_pos_, AutoConfig.kForwardConfig);
    Trajectory t2 = TrajectoryGenerator.generateTrajectory(cube_pos_, new ArrayList<>(), second_score_pos_, AutoConfig.kReverseConfig);
    Trajectory t3 = TrajectoryGenerator.generateTrajectory(second_score_pos_, List.of(interior_waypoint_.getTranslation()), end_pos_, AutoConfig.kReverseConfig);

    addCommands(
      new SequentialCommandGroup(
        new InstantCommand(() -> pose_estimator.resetPosition(start_pos_)),
        new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
        // new TurnDegrees(drivetrain, 180),
        new DriveTrajectory(drivetrain, pose_estimator, () -> t2)),
        new DriveTrajectory(drivetrain, pose_estimator, () -> t3)
      );
      

    //   // Exhaust cube
    //   // superstructure.setPosition(Superstructure.Position.BACK_EXHAUST).withTimeout(2),
    //   // superstructure.setGrabber(() -> 1.0, false).withTimeout(1.0),
      
    //   // Drive to cube pickup while intaking
    //     new ParallelDeadlineGroup(
    //         new SequentialCommandGroup(
    //              new WaitCommand(0.5),
    //              new DriveTrajectory(drivetrain, pose_estimator, () -> t1)
    //           )
    //         // new SequentialCommandGroup(
    //         //       superstructure.setPosition(Superstructure.Position.INTAKE),
    //         //       superstructure.setGrabber(() -> -0.4, true).withTimeout(1.5))
    //           ),
      
    //   new ParallelCommandGroup(
    //       new SequentialCommandGroup(
    //         new TurnDegrees(drivetrain, 180),
    //         new DriveTrajectory(drivetrain, pose_estimator, () -> t2))
    //     //new WaitCommand(0.5),
    //       // superstructure.setPosition(Superstructure.Position.BACK_EXHAUST).withTimeout(2.5)
          
    //   ),
    //   // superstructure.setGrabber(() -> 0.6, false).withTimeout(0.5),

    //   //superstructure.setPosition(Superstructure.Position.STOW),

    //   new ParallelCommandGroup(
    //     // superstructure.setPosition(Superstructure.Position.INTAKE),
    //     new DriveTrajectory(drivetrain, pose_estimator, () -> t3))
    //     // superstructure.setGrabber(() -> -0.4, false).withTimeout(3.5)), 
        
    //     // superstructure.setPosition(Superstructure.Position.STOW)

    // );
  }

    //Flip Poses for Red Alliance
    private static Pose2d mirror(Pose2d pose) {
      return new Pose2d(16.54175 - pose.getX(), pose.getY(),
        new Rotation2d(Math.PI).minus(pose.getRotation()));
    }
}
