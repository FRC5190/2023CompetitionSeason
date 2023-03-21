// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import java.util.ArrayList;

import org.ghrobotics.frc2023.Superstructure;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;
import org.ghrobotics.frc2023.commands.DriveTrajectory;
import org.ghrobotics.frc2023.commands.TurnToDegreesProfiled;

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
public class TwoConeTaxi extends SequentialCommandGroup {
  // Starting Positions (on blue side)
  //CHANGE starting positions for cones
  private static final Pose2d kTopStartingPos = new Pose2d(1.9, 4.5, Rotation2d.fromDegrees(180));
  private static final Pose2d kBotStartingPos = new Pose2d(1.9, 1.071, Rotation2d.fromDegrees(180));
  private static final Pose2d kTopStartingTrajPos = new Pose2d(1.9, 4.5, Rotation2d.fromDegrees(0));
  private static final Pose2d kBotStartingTrajPos = new Pose2d(1.9, 1.071, Rotation2d.fromDegrees(0));
  private static final Pose2d kTopConePos = new Pose2d(6.46, 4.6, Rotation2d.fromDegrees(0));
  private static final Pose2d kBotConePos = new Pose2d(6.46, 0.922,Rotation2d.fromDegrees(0));
  private static final Pose2d kTopConeTrajPos = new Pose2d(6.46, 4.6, Rotation2d.fromDegrees(180));
  private static final Pose2d kBotConeTrajPos = new Pose2d(6.46, 0.922,Rotation2d.fromDegrees(180));
  private static final Pose2d kTopSecConePos = new Pose2d(1.9, 4.5, Rotation2d.fromDegrees(180));
  private static final Pose2d kBotSecConePos = new Pose2d(1.9, 1.071, Rotation2d.fromDegrees(180));
  private static final Pose2d kTopSecTrajPos = new Pose2d(1.9, 4.5, Rotation2d.fromDegrees(0));
  private static final Pose2d kBotSecTrajPos = new Pose2d(1.9, 1.071, Rotation2d.fromDegrees(0));
  private static final Pose2d kTopEndPos = new Pose2d(6.46, 4.6, Rotation2d.fromDegrees(0));;
  private static final Pose2d kBotEndPos = new Pose2d(6.46, 0.922,Rotation2d.fromDegrees(0));
    

  /** Creates a new OneConeTaxi. */
  public TwoConeTaxi(Drivetrain drivetrain, Superstructure superstructure,
  PoseEstimator pose_estimator, DriverStation.Alliance alliance,
  AutoSelector.Grid grid_selection) {
    Pose2d start_pos = grid_selection == AutoSelector.Grid.TOP ? kTopStartingPos : kBotStartingPos;
    Pose2d cone_pos = grid_selection == AutoSelector.Grid.TOP ? kTopConePos : kBotConePos;
    Pose2d start_traj_pos = grid_selection == AutoSelector.Grid.TOP ? kTopStartingTrajPos : kBotStartingTrajPos;
    Pose2d cone_traj_pos = grid_selection == AutoSelector.Grid.TOP ? kTopConeTrajPos : kBotConeTrajPos;
    Pose2d cone_sec_pos = grid_selection == AutoSelector.Grid.TOP ? kTopSecConePos : kBotSecConePos;
    Pose2d sec_traj_pos = grid_selection == AutoSelector.Grid.TOP ? kTopSecTrajPos : kBotSecTrajPos;
    Pose2d end_pos = grid_selection == AutoSelector.Grid.TOP ? kTopEndPos : kBotEndPos;

    boolean should_mirror = alliance == DriverStation.Alliance.Red;

    Pose2d start_pos_ = should_mirror ? mirror(start_pos) : start_pos;
    Pose2d cone_pos_ = should_mirror ? mirror(cone_pos) : cone_pos;
    Pose2d start_traj_pos_ = should_mirror ? mirror(start_traj_pos) : start_traj_pos;
    Pose2d cone_traj_pos_ = should_mirror ? mirror(cone_traj_pos) : cone_traj_pos;
    Pose2d cone_sec_pos_ = should_mirror ? mirror(cone_sec_pos) : cone_sec_pos;
    Pose2d sec_traj_pos_ = should_mirror ? mirror(sec_traj_pos) : sec_traj_pos;
    Pose2d end_pos_ = should_mirror ? mirror(end_pos) : end_pos;


    Trajectory t1 = TrajectoryGenerator.generateTrajectory(start_traj_pos_, new ArrayList<>(), cone_pos_, AutoConfig.kForwardConfig);
    Trajectory t2 = TrajectoryGenerator.generateTrajectory(cone_traj_pos_, new ArrayList<>(), cone_sec_pos_, AutoConfig.kForwardConfig);
    Trajectory t3 = TrajectoryGenerator.generateTrajectory(sec_traj_pos_, new ArrayList<>(), end_pos_, AutoConfig.kForwardConfig);


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> pose_estimator.resetPosition(start_pos_)),
      //superstructure.setPosition(Superstructure.Position.CONE_L2).withTimeout(3.0),
      superstructure.setPosition(Superstructure.Position.CONE_L2_AUTO).withTimeout(3.0),
      //new WaitCommand(2.0),
      superstructure.setGrabber(() -> 0, true).withTimeout(0.5),
      //new WaitCommand(2.0),
      superstructure.setPosition(Superstructure.Position.STOW),
 
      new TurnToDegreesProfiled(Math.toRadians(180), drivetrain),

      new ParallelCommandGroup(
        new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
        superstructure.setPosition(Superstructure.Position.INTAKE).withTimeout(4.0),
        superstructure.setGrabber(() -> 0, true).withTimeout(4.0)
      ),
      
      superstructure.setPosition(Superstructure.Position.STOW),
      new TurnToDegreesProfiled(Math.toRadians(0), drivetrain), 
      new DriveTrajectory(drivetrain, pose_estimator, () -> t2),

      new ParallelCommandGroup(
        superstructure.setPosition(Superstructure.Position.CONE_L2_AUTO).withTimeout(3.0),
        superstructure.setGrabber(() -> 0, true).withTimeout(0.5)
      ), 

      superstructure.setPosition(Superstructure.Position.STOW), 
      new TurnToDegreesProfiled(Math.toRadians(180), drivetrain),
      new DriveTrajectory(drivetrain, pose_estimator, () -> t3)

    );
  }

  //Flip Poses for Red Alliance
  private static Pose2d mirror(Pose2d pose) {
    return new Pose2d(16.54175 - pose.getX(), pose.getY(),
      new Rotation2d(Math.PI).minus(pose.getRotation()));
  }
}
