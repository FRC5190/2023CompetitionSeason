// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ghrobotics.frc2023.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class Telemetry {
    private final ShuffleboardTab tab_;

    private final Drivetrain drivetrain_;
    private final PoseEstimator estimator_;
    private final Limelight limelight_;
    private final Field2d field_;
    private final SendableChooser<Command> auto_selector_;

public Telemetry(Drivetrain drivetrain, PoseEstimator poseEstimator, Limelight limelight, SendableChooser<Command> auto_selector) {
    tab_ = Shuffleboard.getTab("2023");

    drivetrain_ = drivetrain;
    estimator_ = poseEstimator;
    limelight_ = limelight;
    auto_selector_ = auto_selector;

    field_ = new Field2d();
    SmartDashboard.putData("Field", field_);

    // Put autonomous mode selector on Shuffleboard.
    tab_.add("Autonomous Mode Selector", auto_selector_)
        .withSize(3, 2)
        .withPosition(3, 3);

    ShuffleboardLayout drivetrain_layout = tab_.getLayout("Drivetrain", BuiltInLayouts.kGrid)
        .withSize(2,2)
        .withPosition(1,0);
    drivetrain_layout.addNumber("L Position (m)", drivetrain::getLeftPosition)
        .withPosition(0, 0);
    drivetrain_layout.addNumber("R Position (m)", drivetrain::getRightPosition)
        .withPosition(0, 1);

    ShuffleboardLayout vision_layout = tab_.getLayout("Limelight", BuiltInLayouts.kGrid)
        .withSize(5,2)
        .withPosition(1, 3);
    vision_layout.addNumber("Cam X", estimator_::getCamX)
        .withPosition(0,0);
    vision_layout.addNumber("Cam Y", estimator_::getCamY)
        .withPosition(0,1);
    vision_layout.addNumber("X offset", limelight_::getTx)
        .withPosition(0,2);
    vision_layout.addNumber("Y offset", limelight_::getTy)
        .withPosition(1, 0);
    vision_layout.addNumber("Vision Estimate X", 
        () -> estimator_.getCurrentPose().getX())
        .withPosition(2, 0);
    vision_layout.addNumber("Vision Estimate Y", 
        () -> estimator_.getCurrentPose().getY())
        .withPosition(2, 1);
    vision_layout.addNumber("Vision Estimate Angle", 
        () -> estimator_.getCurrentPose().getRotation().getDegrees())
        .withPosition(2, 2);
    vision_layout.addNumber("April Tag ID", 
    () -> estimator_.getIDValue())
        .withPosition(3,1);
}

public void periodic(){
    Pose2d robot_pose = estimator_.getCurrentPose();
    field_.setRobotPose(robot_pose);
    field_.getObject("Drivetrain").setPose(
        robot_pose.transformBy(
            new Transform2d(new Translation2d(), estimator_.getCurrentPose().getRotation())));
}
}
