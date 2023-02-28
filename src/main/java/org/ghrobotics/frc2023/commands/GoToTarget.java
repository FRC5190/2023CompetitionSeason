// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.commands;

import org.ghrobotics.frc2023.auto.AutoConfig;

import org.ghrobotics.frc2023.Arena;
import org.ghrobotics.frc2023.Limelight;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.Gyroscope;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

public class GoToTarget extends CommandBase {
  private final Drivetrain drivetrain_;
  private final PoseEstimator pose_estimator_;
  private final Gyroscope gyroscope_;
  private final Limelight limelight_;
  private final String target_side_choice_;
  private final Arena arena_;
  private Pose2d end_position_;
  private Pose2d start_position_;

  /** Creates a new GoToTarget. */
  public GoToTarget(PoseEstimator poseEstimator, Drivetrain drivetrain, Gyroscope gyroscope, String sideChoice, Limelight limelight, Arena arena) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain_ = drivetrain;
    pose_estimator_ = poseEstimator;
    gyroscope_ = gyroscope;
    limelight_ = limelight;
    target_side_choice_ = sideChoice;
    arena_ = arena;


    addRequirements(drivetrain_);
    addRequirements(pose_estimator_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (limelight_.hasTarget()) {
      int tag_id = (int) limelight_.getID();
      Pose3d tagPose = arena_.blueLayout.getTagPose(tag_id).get();

      if (target_side_choice_ == "Left") {
        end_position_ = tagPose.transformBy(Arena.blueTransform[0]).toPose2d();
      } else if (target_side_choice_ == "Center") {
        end_position_ = tagPose.transformBy(Arena.blueTransform[1]).toPose2d();
      } else if (target_side_choice_ == "Right") {
        end_position_ = tagPose.transformBy(Arena.blueTransform[2]).toPose2d();
      }
      
      start_position_ = pose_estimator_.getCurrentPose();

      SmartDashboard.putNumber("Start Position (X)", start_position_.getX());
      SmartDashboard.putNumber("Start Position (Y)", start_position_.getY());
      SmartDashboard.putNumber("End Position (X)", end_position_.getX());
      SmartDashboard.putNumber("End Position (Y)", end_position_.getY());

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new DriveTrajectory(drivetrain_, pose_estimator_, TrajectoryGenerator.generateTrajectory
      (start_position_, List.of(), end_position_, AutoConfig.kForwardConfig));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
