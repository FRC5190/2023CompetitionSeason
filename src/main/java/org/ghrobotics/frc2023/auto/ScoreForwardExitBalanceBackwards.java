package org.ghrobotics.frc2023.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.ArrayList;
import java.util.List;
import org.ghrobotics.frc2023.Superstructure;
import org.ghrobotics.frc2023.commands.DriveBalance;
import org.ghrobotics.frc2023.commands.DriveTrajectory;
import org.ghrobotics.frc2023.commands.TurnToAngle;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

//Score cube (high) forwards
// * score L3 cube forward
// * move backward out of the community
// * go to the charge station waypoint backwards to align with the middle of the charge station
// * go onto the charge station
// * run the drive balance command

public class ScoreForwardExitBalanceBackwards extends SequentialCommandGroup {
  // Starting Positions (on blue side)
  private static final Pose2d kBotStartingPos = new Pose2d(1.900, 1.071, Rotation2d.fromDegrees(180));
  private static final Pose2d kTopStartingPos = new Pose2d(1.900, 4.424, Rotation2d.fromDegrees(180));

  // Cube Positions (on blue side)
  private static final Pose2d kBotCubeWaypoint = new Pose2d(5.5, 0.922, Rotation2d.fromDegrees(180));
  private static final Pose2d kTopCubeWaypoint = new Pose2d(5.5, 4.589, Rotation2d.fromDegrees(180));

  // Charge Station Positions
  private static final Pose2d kChargeStationWaypoint = new Pose2d(6.294, 2.800, Rotation2d.fromDegrees(0));
  private static final Pose2d kChargeStation = new Pose2d(4.594, 2.900, Rotation2d.fromDegrees(0));

  // Constructor
  public ScoreForwardExitBalanceBackwards(Drivetrain drivetrain, Superstructure superstructure,
                                 PoseEstimator pose_estimator, DriverStation.Alliance alliance,
                                 AutoSelector.Grid grid_selection) {

    //Assign top or bottom position
    Pose2d start_pos = grid_selection == AutoSelector.Grid.TOP ? kTopStartingPos : kBotStartingPos;
    Pose2d cube_pos = grid_selection == AutoSelector.Grid.TOP ? kTopCubeWaypoint : kBotCubeWaypoint;

    // Check if we need to mirror poses
    boolean should_mirror = alliance == DriverStation.Alliance.Red;

    // Get starting, cube, and charge station positions
    Pose2d start_pos_ = should_mirror ? mirror(start_pos) : start_pos;
    cube_pos = should_mirror ? mirror(cube_pos) : cube_pos;
    Pose2d charge_station_w_pos = should_mirror ? mirror(
        kChargeStationWaypoint) : kChargeStationWaypoint;
    Pose2d charge_station_pos = should_mirror ? mirror(kChargeStation) : kChargeStation;

    // Generate trajectory from start pos to cube pos
    Trajectory t1 = TrajectoryGenerator.generateTrajectory(
        start_pos_, List.of(cube_pos.getTranslation(), charge_station_w_pos.getTranslation()), 
        charge_station_pos, AutoConfig.kReverseConfig);

    // Generate trajectory from cube pos to charge station
  /*   Trajectory t1 = TrajectoryGenerator.generateTrajectory(
        start_pos, List.of(cube_pos.getTranslation(), charge_station_w_pos.getTranslation()), charge_station_pos,
        AutoConfig.kReverseToBalanceConfig);
    */
        Trajectory t2 = TrajectoryGenerator.generateTrajectory(
          cube_pos, List.of(charge_station_w_pos.getTranslation()), charge_station_pos,
          AutoConfig.kReverseToBalanceConfig);
  
    // Add commands
    addCommands(
      new WaitCommand(0.3),

        // Reset pose estimator to starting position
        new InstantCommand(() -> pose_estimator.resetPosition(start_pos_)),

        // Exhaust cube
        superstructure.setPosition(Superstructure.Position.CUBE_L3),
        superstructure.setGrabber(() -> 0.3, false).withTimeout(0.5),

        new ParallelCommandGroup(
        //new TurnToAngle(180, drivetrain, pose_estimator),
          new DriveTrajectory(drivetrain, pose_estimator, () -> t1),
          superstructure.setPosition(Superstructure.Position.STOW)
        ),

        //new DriveTrajectory(drivetrain, pose_estimator, () -> t2),
        // Balance
        new DriveBalance(drivetrain)
    );
  }
      //Flip Poses for Red Alliance
      private static Pose2d mirror(Pose2d pose) {
        return new Pose2d(16.54175 - pose.getX(), pose.getY(),
          new Rotation2d(Math.PI).minus(pose.getRotation()));
      }

}
