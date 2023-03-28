package org.ghrobotics.frc2023.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

public class DriveTowardPosition extends CommandBase {
  // Subsystems and Controllers
  private final Drivetrain drivetrain_;
  private final PoseEstimator pose_estimator_;
  private final CommandXboxController controller_;

  // Target
  private final Pose2d target_;

  // PID Controller
  private final PIDController turn_fb_;

  // Constructor
  public DriveTowardPosition(Drivetrain drivetrain, PoseEstimator pose_estimator,
                             CommandXboxController controller, Pose2d target) {
    // Assign member variables
    drivetrain_ = drivetrain;
    pose_estimator_ = pose_estimator;
    controller_ = controller;
    target_ = target;

    // Initialize PID controller
    turn_fb_ = new PIDController(Constants.kP, 0, Constants.kD);

    // Add subsystem requirements
    addRequirements(drivetrain_);
  }

  @Override
  public void initialize() {
    // Reset PID controller.
    turn_fb_.reset();
    turn_fb_.setSetpoint(0);
  }

  @Override
  public void execute() {
    // Get requested forward component
    double fwd = -controller_.getLeftY();

    // Create baseline turn value
    double turn = 0;

    // Calculate goal position in robot's coordinates.
    Pose2d robot_to_goal = target_.relativeTo(pose_estimator_.getPosition());

    // Calculate angle to goal.
    Rotation2d angle_to_goal = robot_to_goal.getTranslation().getAngle();

    // Do some sanity checks on the angle to make sure it isn't bogus.
    if (Math.abs(angle_to_goal.getRadians()) < Constants.kAngleTolerance) {
      turn = turn_fb_.calculate(angle_to_goal.getRadians());
    }

    // Set percent
    drivetrain_.setPercent(fwd + turn, fwd - turn);
  }

  // Constants
  private static class Constants {
    // Tolerances
    public static final double kAngleTolerance = Math.toRadians(45);

    // Feedback (values from Crossfire 2019)
    public static final double kP = 0.8;
    public static final double kD = 0.0;
  }
}
