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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneConeTaxi extends SequentialCommandGroup {
  // Starting Positions (on blue side)
  //CHANGE starting positions for cones
  private static final Pose2d kTopStartingPos = new Pose2d(1.9, 4.5, Rotation2d.fromDegrees(180));
  private static final Pose2d kBotStartingPos = new Pose2d(1.9, 1.071, Rotation2d.fromDegrees(180));
  private static final Pose2d kTopConePos = new Pose2d(6.46, 4.6, Rotation2d.fromDegrees(180));
  private static final Pose2d kBotConePos = new Pose2d(6.46, 0.922,Rotation2d.fromDegrees(180));
  

  /** Creates a new OneConeTaxi. */
  public OneConeTaxi(Drivetrain drivetrain, Superstructure superstructure,
  PoseEstimator pose_estimator, DriverStation.Alliance alliance,
  AutoSelector.Grid grid_selection) {
    Pose2d start_pos = grid_selection == AutoSelector.Grid.TOP ? kTopStartingPos : kBotStartingPos;
    Pose2d cone_pos = grid_selection == AutoSelector.Grid.TOP ? kTopConePos : kBotConePos;

    boolean should_mirror = alliance == DriverStation.Alliance.Red;

    Pose2d start_pos_ = should_mirror ? mirror(start_pos) : start_pos;
    Pose2d cone_pos_ = should_mirror ? mirror(cone_pos) : cone_pos;

    Trajectory t1 = TrajectoryGenerator.generateTrajectory(start_pos_, new ArrayList<>(), cone_pos_, AutoConfig.kForwardConfig);


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> pose_estimator.resetPosition(start_pos_)),
      superstructure.setPosition(Superstructure.Position.CONE_L2).withTimeout(3.0),
      //superstructure.setPosition(Superstructure.Position.CONE_L2_AUTO).withTimeout(3.0),
      //new WaitCommand(2.0),
      superstructure.setGrabber(() -> 0, true).withTimeout(0.5),
      //new WaitCommand(2.0),
      superstructure.setPosition(Superstructure.Position.STOW)

      //new TurnToDegreesProfiled(Math.toRadians(180), drivetrain)
      //new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
      //new TurnToDegreesProfiled(Math.toRadians(180), drivetrain)

    );
  }

  //Flip Poses for Red Alliance
  private static Pose2d mirror(Pose2d pose) {
    return new Pose2d(16.54175 - pose.getX(), pose.getY(),
      new Rotation2d(Math.PI).minus(pose.getRotation()));
  }
}
