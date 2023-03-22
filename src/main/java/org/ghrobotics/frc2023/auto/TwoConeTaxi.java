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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.ArrayList;
import org.ghrobotics.frc2023.Superstructure;
import org.ghrobotics.frc2023.commands.DriveTrajectory;
import org.ghrobotics.frc2023.commands.TurnToDegreesProfiled;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoConeTaxi extends SequentialCommandGroup {
  // Starting Positions (on blue side)
  private static final Pose2d kTopSecConePos = new Pose2d(1.85, 4.95, Rotation2d.fromDegrees(180));
  private static final Pose2d kBotSecConePos = new Pose2d(1.85, 0.53, Rotation2d.fromDegrees(180));

  private static final Pose2d kTopConePos = new Pose2d(5.7, 4.638, Rotation2d.fromDegrees(0)); //6.95, .89
  private static final Pose2d kBotConePos = new Pose2d(5.7, 0.927, Rotation2d.fromDegrees(0));

  private static final Pose2d kTopStartingPos = new Pose2d(1.9, 4.5, Rotation2d.fromDegrees(180));
  private static final Pose2d kBotStartingPos = new Pose2d(1.9, 1.633, Rotation2d.fromDegrees(180));

  /**
   * Creates a new OneConeTaxi.
   */
  public TwoConeTaxi(Drivetrain drivetrain, Superstructure superstructure,
                     PoseEstimator pose_estimator, DriverStation.Alliance alliance,
                     AutoSelector.Grid grid_selection) {

    Pose2d start_pos = grid_selection == AutoSelector.Grid.TOP ? kTopStartingPos : kBotStartingPos;
    Pose2d cone_pos = grid_selection == AutoSelector.Grid.TOP ? kTopConePos : kBotConePos;
    Pose2d cone_sec_pos = grid_selection == AutoSelector.Grid.TOP ? kTopSecConePos : kBotSecConePos;

    boolean should_mirror = alliance == DriverStation.Alliance.Red;

    start_pos = should_mirror ? mirror(start_pos) : start_pos;
    cone_pos = should_mirror ? mirror(cone_pos) : cone_pos;
    cone_sec_pos = should_mirror ? mirror(cone_sec_pos) : cone_sec_pos;


    Trajectory t1 = TrajectoryGenerator.generateTrajectory(start_pos.transformBy(
            new Transform2d(new Translation2d(), Rotation2d.fromDegrees(140))), new ArrayList<>(),
        cone_pos, AutoConfig.kForwardConfig);
    Trajectory t2 = TrajectoryGenerator.generateTrajectory(cone_pos.transformBy(
            new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180))), new ArrayList<>(),
        cone_sec_pos, AutoConfig.kForwardConfig);

    Pose2d finalStart_pos = start_pos;
    addCommands(
        new WaitCommand(0.5),
        new InstantCommand(() -> pose_estimator.resetPosition(finalStart_pos)),
        superstructure.setPosition(Superstructure.Position.CONE_L2),
        superstructure.setGrabber(() -> 0, true).withTimeout(0.5),

        new ParallelCommandGroup(
            superstructure.setPosition(Superstructure.Position.STOW),
            new SequentialCommandGroup(
                new RunCommand(() -> drivetrain.setPercent(-0.1, -0.1), drivetrain).withTimeout(
                    0.5),
                new TurnToDegreesProfiled(Math.toRadians(-40), drivetrain, pose_estimator)
            )
        ),

        new ParallelDeadlineGroup(
            new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
            superstructure.setPosition(Superstructure.Position.INTAKE),
            superstructure.setGrabber(() -> 0.1, true)
        ),

        new ParallelCommandGroup(
            superstructure.setPosition(Superstructure.Position.STOW),
            new SequentialCommandGroup(
                new TurnToDegreesProfiled(Math.toRadians(0), drivetrain, pose_estimator),
                new DriveTrajectory(drivetrain, pose_estimator, () -> t2)
            )
        ),

        superstructure.setPosition(Superstructure.Position.CONE_L2_AUTO),
        superstructure.setGrabber(() -> 0, true).withTimeout(0.5),
        superstructure.setPosition(Superstructure.Position.STOW)
    );
  }

  //Flip Poses for Red Alliance
  private static Pose2d mirror(Pose2d pose) {
    return new Pose2d(16.54175 - pose.getX(), pose.getY(),
        new Rotation2d(Math.PI).minus(pose.getRotation()));
  }
}
