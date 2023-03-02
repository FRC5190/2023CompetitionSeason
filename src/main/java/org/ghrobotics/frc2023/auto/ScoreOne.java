package org.ghrobotics.frc2023.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.ghrobotics.frc2023.Arena;
import org.ghrobotics.frc2023.commands.DriveTrajectory;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

public class ScoreOne extends SequentialCommandGroup {
  // Constructor
  public ScoreOne(Drivetrain drivetrain, PoseEstimator pose_estimator,
                  AutoSelector.Side side, AutoSelector.Height height,
                  AutoSelector.Balance balance) {
    // Trajectory
    AtomicReference<Trajectory> trajectory = new AtomicReference<>(new Trajectory());

    // Pick what transform to use
    Transform3d transform = null;
    switch (side) {
      case LEFT:
        transform = Arena.getTagToLPosition();
        break;
      case RIGHT:
        transform = Arena.getTagToRPosition();
        break;
      case CENTER:
        transform = Arena.getTagToCPosition();
        break;
    }

    // Store in final variable to bypass Java restrictions.
    final Transform3d f_transform = transform;

    // Create a master command group that will run when this auto is enabled.
    addCommands(
        // Reset the robot's position based on our current vision measurements.
        new InstantCommand(pose_estimator::resetPositionFromVision),

        // DEBUG
        new InstantCommand(
            () -> pose_estimator.resetPosition(new Pose2d(5.2, 1.27, Rotation2d.fromDegrees(180)))),

        // Generate a trajectory based on current position and where we want to go.
        new InstantCommand(() -> {
          // We want to be a bit more careful about generating trajectories on the fly
          // and catch exceptions so that the robot code does not crash.
          try {
            // Get current position
            Pose2d s = pose_estimator.getPosition();
            System.out.println(s);

            // Calculate end position from tag and transform
            Pose2d e = Arena.getTagPosition(8).transformBy(
                f_transform).toPose2d();
            System.out.println(e);

            // Generate trajectory
            trajectory.set(TrajectoryGenerator.generateTrajectory(s, List.of(), e,
                AutoConfig.kForwardConfig));
          } catch (Exception ignored) {
            System.out.println("An exception was thrown during dynamic trajectory generation!");
          }
        }),

        // Drive to target and take superstructure to correct position
        new ParallelCommandGroup(
            new DriveTrajectory(drivetrain, pose_estimator, trajectory::get)
            // Superstructure
        )

        // Place game piece
    );
  }
}
