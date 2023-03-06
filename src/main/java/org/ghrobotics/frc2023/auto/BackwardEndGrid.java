package org.ghrobotics.frc2023.auto;

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
import java.util.ArrayList;
import org.ghrobotics.frc2023.Superstructure;
import org.ghrobotics.frc2023.commands.DriveBalance;
import org.ghrobotics.frc2023.commands.DriveTrajectory;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

// Backward End Grid:
//  * line up backward at one of the end grids (aligned to center)
//  * shoot cube into L3
//  * pick up cube
//  * back into charge station and balance
public class BackwardEndGrid extends SequentialCommandGroup {
  // Starting Positions (on blue side)
  private static final Pose2d kBotStartingPos = new Pose2d(1.900, 1.071, new Rotation2d());
  private static final Pose2d kTopStartingPos = new Pose2d(1.900, 4.424, new Rotation2d());

  // Cube Positions (on blue side)
  private static final Pose2d kBotCube = new Pose2d(6.741, 0.922, new Rotation2d());
  private static final Pose2d kTopCube = new Pose2d(6.741, 4.589, new Rotation2d());

  // Charge Station Positions
  private static final Pose2d kChargeStation = new Pose2d(5.711, 2.700, new Rotation2d());

  // Constructor
  public BackwardEndGrid(Drivetrain drivetrain, Superstructure superstructure,
                         PoseEstimator pose_estimator, DriverStation.Alliance alliance) {
    // Check if we need to mirror poses
    boolean should_mirror = alliance == DriverStation.Alliance.Red;

    // Get starting, cube, and charge station positions
    Pose2d start_pos = should_mirror ? mirror(kBotStartingPos) : kBotStartingPos;
    Pose2d cube_pos = should_mirror ? mirror(kBotCube) : kBotCube;
    Pose2d charge_station_pos = should_mirror ? mirror(kChargeStation) : kChargeStation;

    // Generate trajectory from start pos to cube pos
    Trajectory t1 = TrajectoryGenerator.generateTrajectory(
        start_pos, new ArrayList<>(), cube_pos,
        AutoConfig.kForwardConfig);

    // Generate trajectory from cube pos to charge station
    Trajectory t2 = TrajectoryGenerator.generateTrajectory(
        cube_pos, new ArrayList<>(), charge_station_pos,
        AutoConfig.kReverseConfig);

    // Add commands
    addCommands(
        // Reset pose estimator to starting position
        new InstantCommand(() -> pose_estimator.resetPosition(start_pos)),

        // Exhaust cube
        superstructure.setPosition(Superstructure.Position.BACK_EXHAUST),

        // Drive to cube pickup while intaking
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new WaitCommand(1.5),
                new DriveTrajectory(drivetrain, pose_estimator, () -> t1)),
            superstructure.setPosition(Superstructure.Position.INTAKE)),

        // Drive to charge station
        new ParallelCommandGroup(
            new DriveTrajectory(drivetrain, pose_estimator, () -> t2),
            superstructure.setPosition(Superstructure.Position.STOW)
        ),

        // Balance
        new DriveBalance(drivetrain)
    );
  }

  // Flip Poses for Red Alliance
  private static Pose2d mirror(Pose2d pose) {
    return new Pose2d(16.54175 - pose.getX(), pose.getY(),
        new Rotation2d(Math.PI).minus(pose.getRotation()));
  }
}
