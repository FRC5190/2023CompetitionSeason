// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ghrobotics.frc2023.auto.AutoSelector;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.Limelight;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

/**
 * Add your docs here.
 */
public class Telemetry {
  // Shuffleboard Tab
  private final ShuffleboardTab tab_;

  // Subsystems
  private final Drivetrain drivetrain_;
  private final PoseEstimator pose_estimator_;
  private final Limelight limelight_;

  // Auto Selector
  private final AutoSelector auto_selector_;

  // Visualizations
  private final Field2d field_;

  public Telemetry(Drivetrain drivetrain, PoseEstimator pose_estimator, Limelight limelight,
                   AutoSelector auto_selector) {

    // Assign member variables
    drivetrain_ = drivetrain;
    pose_estimator_ = pose_estimator;
    limelight_ = limelight;
    auto_selector_ = auto_selector;

    // Get Shuffleboard tab that we want everything in
    tab_ = Shuffleboard.getTab("2023");

    // Initialize field visualization
    field_ = new Field2d();
    SmartDashboard.putData("Field", field_);

    // Put autonomous mode selector on Shuffleboard.
    tab_.add("Autonomous Side Selector", auto_selector_.getSideChooser())
        .withSize(3, 2)
        .withPosition(3, 3);

    tab_.add("Autonomous Height Selector", auto_selector_.getHeightChooser())
        .withSize(3, 2)
        .withPosition(4, 4);
    tab_.add("Balance Choice", auto_selector_.getBalanceChooser())
        .withSize(3, 2)
        .withPosition(3, 1);

    // Add drivetrain information
    ShuffleboardLayout drivetrain_layout = tab_.getLayout("Drivetrain", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(1, 0);
    drivetrain_layout.addNumber("L Position (m)", drivetrain::getLeftPosition)
        .withPosition(0, 0);
    drivetrain_layout.addNumber("R Position (m)", drivetrain::getRightPosition)
        .withPosition(0, 1);
    drivetrain_layout.addNumber("L Velocity (mps)", drivetrain::getLeftVelocity)
        .withPosition(1, 0);
    drivetrain_layout.addNumber("R Velocity (mps)", drivetrain::getRightVelocity)
        .withPosition(1, 1);

    // Add vision information
    ShuffleboardLayout vision_layout = tab_.getLayout("Limelight", BuiltInLayouts.kGrid)
        .withSize(5, 2)
        .withPosition(1, 3);
    vision_layout.addNumber("X offset", limelight_::getTx)
        .withPosition(0, 2);
    vision_layout.addNumber("Y offset", limelight_::getTy)
        .withPosition(1, 0);
    vision_layout.addNumber("Vision Estimate X",
            () -> pose_estimator_.getPosition().getX())
        .withPosition(2, 0);
    vision_layout.addNumber("Vision Estimate Y",
            () -> pose_estimator_.getPosition().getY())
        .withPosition(2, 1);
    vision_layout.addNumber("Vision Estimate Angle",
            () -> pose_estimator_.getPosition().getRotation().getDegrees())
        .withPosition(2, 2);
    vision_layout.addNumber("April Tag ID",
            () -> pose_estimator_.getVisionPrimaryTagId())
        .withPosition(3, 1);
  }

  public void periodic() {
    Pose2d robot_pose = pose_estimator_.getPosition();
    field_.setRobotPose(robot_pose);
  }
}
