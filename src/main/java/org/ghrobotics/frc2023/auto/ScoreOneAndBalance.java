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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneAndBalance extends SequentialCommandGroup {
    private static final Pose2d kBotStartingPos = new Pose2d(1.9, 3.5, new Rotation2d());
    private static final Pose2d kOutOfCommunityPos = new Pose2d(5.5, 3.0, new Rotation2d());
    private static final Pose2d kOnChargeStation = new Pose2d(5.0, 3.0, new Rotation2d());

  /** Creates a new ScoreOneAndBalance. */
  public ScoreOneAndBalance(Drivetrain drivetrain, Superstructure superstructure, 
                            PoseEstimator pose_estimator, DriverStation.Alliance alliance) {

       // Check if we need to mirror poses
       boolean should_mirror = alliance == DriverStation.Alliance.Red;

       // Get starting positions
       //Pose2d kBotStartingPos = pose_estimator.getPosition();
       Pose2d start_pos = should_mirror ? mirror(kBotStartingPos) : kBotStartingPos;
       Pose2d out_of_comm = should_mirror ? mirror(kOutOfCommunityPos) : kOutOfCommunityPos;
       Pose2d on_charge_station = should_mirror ? mirror(kOnChargeStation) : kOnChargeStation;


       Trajectory t1 = TrajectoryGenerator.generateTrajectory(start_pos, new ArrayList<>(), out_of_comm, AutoConfig.kForwardConfig);
       Trajectory t2 = TrajectoryGenerator.generateTrajectory(out_of_comm, new ArrayList<>(), on_charge_station, AutoConfig.kReverseConfig);

    addCommands(
      new InstantCommand(() -> pose_estimator.resetPosition(start_pos)),

      superstructure.setPosition(Superstructure.Position.BACK_EXHAUST).withTimeout(2),
      superstructure.setGrabber(() -> 0.35, false).withTimeout(0.5),

      new ParallelCommandGroup(
        new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
        superstructure.setPosition(Superstructure.Position.STOW)
      ),

      new DriveTrajectory(drivetrain, pose_estimator, () -> t2),

      new DriveBalance(drivetrain)

    );
  }

  //Flip Poses for Red Alliance
  private static Pose2d mirror(Pose2d pose) {
    return new Pose2d(16.54175 - pose.getX(), pose.getY(),
      new Rotation2d(Math.PI).minus(pose.getRotation()));
  }

}
