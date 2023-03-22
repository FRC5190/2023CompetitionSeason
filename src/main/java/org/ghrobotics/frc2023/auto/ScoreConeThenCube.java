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
import java.util.List;
import org.ghrobotics.frc2023.Superstructure;
import org.ghrobotics.frc2023.commands.DriveTrajectory;
import org.ghrobotics.frc2023.commands.TurnToAngle;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

// ScoreConeThenCube
//  * score cone on top grid
//  * pick up cube
//  * score cube on top grid
public class ScoreConeThenCube extends SequentialCommandGroup {
  // Poses (assuming origin on blue side)
  private static final Pose2d kStartPos = new Pose2d(1.90, 5.00, new Rotation2d(Math.PI));
  private static final Pose2d kCubePickupPos = new Pose2d(6.34, 4.75, new Rotation2d());
  private static final Pose2d kCubeScorePos = new Pose2d(1.90, 4.52, Rotation2d.fromDegrees(175));
  private static final Pose2d kIntermediatePos = new Pose2d(3.9, 4.785, new Rotation2d());

  // Constructor
  public ScoreConeThenCube(Drivetrain drivetrain, Superstructure superstructure,
                           PoseEstimator pose_estimator, DriverStation.Alliance alliance) {

    // Get true poses from alliance
    Pose2d start_pos = AutoConfig.adjustPoseForAlliance(kStartPos, alliance);
    Pose2d cube_pickup_pos = AutoConfig.adjustPoseForAlliance(kCubePickupPos, alliance);
    Pose2d cube_score_pos = AutoConfig.adjustPoseForAlliance(kCubeScorePos, alliance);
    Pose2d intermediate_pos = AutoConfig.adjustPoseForAlliance(kIntermediatePos, alliance);

    // Calculate angles to turn to
    double angle1 = alliance == DriverStation.Alliance.Red ? 160 : 20;
    double angle2 = alliance == DriverStation.Alliance.Red ? 0 : 180;

    // Create 180 deg transform
    Transform2d turn_transform = new Transform2d(new Translation2d(), new Rotation2d(Math.PI));

    // Create trajectory from starting pos to cube pickup
    Trajectory t1 = TrajectoryGenerator.generateTrajectory(
        start_pos.transformBy(turn_transform), List.of(), cube_pickup_pos,
        AutoConfig.kForwardConfig);

    // Create trajectory from cube pickup to cube scoring pos
    Trajectory t2 = TrajectoryGenerator.generateTrajectory(
        cube_pickup_pos.transformBy(turn_transform), List.of(intermediate_pos.getTranslation()),
        cube_score_pos, AutoConfig.kForwardConfig);

    // Add commands
    addCommands(
        new WaitCommand(0.5), // TODO: figure out why this is needed

        // Score cone in L2:
        new InstantCommand(() -> pose_estimator.resetPosition(start_pos)),
        superstructure.setPosition(Superstructure.Position.CONE_L2),
        superstructure.setGrabber(0.0, true).withTimeout(0.3),

        // Pickup cube:
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new RunCommand(() -> drivetrain.setPercent(-0.1, -0.3), drivetrain)
                    .withTimeout(0.5),
                new TurnToAngle(Math.toRadians(angle1), drivetrain, pose_estimator),
                new DriveTrajectory(drivetrain, pose_estimator, t1)
            ),
            new SequentialCommandGroup(
                superstructure.setPosition(Superstructure.Position.STOW),
                superstructure.setPosition(Superstructure.Position.INTAKE),
                superstructure.setGrabber(-0.5, true)
            )
        ),

        // Score cube:
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new TurnToAngle(Math.toRadians(angle2), drivetrain, pose_estimator),
                new DriveTrajectory(drivetrain, pose_estimator, t2)
            ),
            new SequentialCommandGroup(
                superstructure.setPosition(Superstructure.Position.STOW).withTimeout(0.1),
                new WaitCommand(t2.getTotalTimeSeconds() - 0.5),
                superstructure.setPosition(Superstructure.Position.CUBE_L3),
                superstructure.setGrabber(0.75, false).withTimeout(0.5)
            )
        ),

        // Stow superstructure:
        superstructure.setPosition(Superstructure.Position.STOW)
    );
  }
}
