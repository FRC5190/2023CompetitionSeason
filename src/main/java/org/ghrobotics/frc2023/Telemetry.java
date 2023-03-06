// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.ArrayList;
import java.util.List;
import org.ghrobotics.frc2023.auto.AutoSelector;
import org.ghrobotics.frc2023.subsystems.Arm;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.Elevator;
import org.ghrobotics.frc2023.subsystems.Extender;
import org.ghrobotics.frc2023.subsystems.Limelight;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

public class Telemetry {

  // Periodic
  private final List<Runnable> periodic_registry_ = new ArrayList<>();

  // Visualizations
  private final Field2d field_;
  private final Mechanism2d superstructure_;

  public Telemetry(Drivetrain drivetrain, Elevator elevator, Extender extender, Arm arm,
                   PoseEstimator pose_estimator, Limelight limelight,
                   AutoSelector auto_selector) {
    // Get Shuffleboard tab that we want everything in
    ShuffleboardTab tab_ = Shuffleboard.getTab("2023");

    // Initialize field visualization
    field_ = new Field2d();
    tab_.add("Field", field_);
    periodic_registry_.add(() -> field_.setRobotPose(pose_estimator.getPosition()));

    // Initialize mechanism visualization
    superstructure_ = new Mechanism2d(1.2, 1.2);
    tab_.add("Superstructure", superstructure_);

    // Add elevator, extender and arm to visualization
    superstructure_.getRoot("Structure Root", 0.33, 0.1).append(
        new MechanismLigament2d("Structure", 0.79, 90));

    MechanismLigament2d carriage = superstructure_.getRoot("Carriage Root", 0.35, 0.1).append(
        new MechanismLigament2d("Carriage", 0.02, 90, 6, new Color8Bit(Color.kAqua)));
    MechanismLigament2d extension_base = carriage.append(
        new MechanismLigament2d("Extender Base", 0.1, -90, 4, new Color8Bit(Color.kYellow)));
    MechanismLigament2d extension = extension_base.append(
        new MechanismLigament2d("Extender", 0, 0, 4, new Color8Bit(Color.kLimeGreen)));
    MechanismLigament2d arm_out = extension.append(
        new MechanismLigament2d("Arm", 0.15, 0, 3, new Color8Bit(Color.kOrangeRed)));

    // Update visualizations
    periodic_registry_.add(() -> carriage.setLength(0.02 + elevator.getPosition()));
    periodic_registry_.add(() -> extension.setLength(0.02 + extender.getPosition()));
    periodic_registry_.add(() -> arm_out.setAngle(Math.toDegrees(arm.getAngle())));

    // Put autonomous mode selector on Shuffleboard.
    ShuffleboardLayout auto_layout = tab_.getLayout("Autonomous", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withPosition(0, 0);
    auto_layout.add("Side", auto_selector.getSideChooser())
        .withSize(2, 1);
    auto_layout.add("Height", auto_selector.getHeightChooser())
        .withSize(2, 1);
    auto_layout.add("Balance", auto_selector.getBalanceChooser())
        .withSize(2, 1);

    // Add pose estimator information
    ShuffleboardLayout pose_estimator_layout = tab_.getLayout("Pose Estimator",
            BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(2, 0);
    pose_estimator_layout.addNumber("Robot X (ft)",
            () -> Units.metersToFeet(pose_estimator.getPosition().getX()))
        .withPosition(0, 0);
    pose_estimator_layout.addNumber("Robot Y (ft)",
            () -> Units.metersToFeet(pose_estimator.getPosition().getY()))
        .withPosition(1, 0);
    pose_estimator_layout.addNumber("Robot A (deg)",
            () -> pose_estimator.getPosition().getRotation().getDegrees())
        .withPosition(0, 1);
    pose_estimator_layout.addNumber("Tag ID", pose_estimator::getVisionPrimaryTagId)
        .withPosition(1, 1);

    // Add drivetrain information
    ShuffleboardLayout drivetrain_layout = tab_.getLayout("Drivetrain", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(4, 0);
    drivetrain_layout.addNumber("L Position (m)", drivetrain::getLeftPosition)
        .withPosition(0, 0);
    drivetrain_layout.addNumber("R Position (m)", drivetrain::getRightPosition)
        .withPosition(0, 1);
    drivetrain_layout.addNumber("L Velocity (mps)", drivetrain::getLeftVelocity)
        .withPosition(1, 0);
    drivetrain_layout.addNumber("R Velocity (mps)", drivetrain::getRightVelocity)
        .withPosition(1, 1);
    drivetrain_layout.addNumber("Pitch (deg)", () -> Math.toDegrees(drivetrain.getPitch()))
        .withPosition(0, 2);

    // Add elevator information
    ShuffleboardLayout elevator_layout = tab_.getLayout("Elevator", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(6, 0);
    elevator_layout.addNumber("Position (in)", () -> Units.metersToInches(elevator.getPosition()))
        .withPosition(0, 0);
    elevator_layout.addNumber("Velocity (ips)", () -> Units.metersToInches(elevator.getVelocity()))
        .withPosition(1, 0);
    elevator_layout.addNumber("Velocity Setpoint (ips)",
            () -> Units.metersToInches(elevator.getVelocitySetpoint()))
        .withPosition(0, 1);

    // Add arm information
    ShuffleboardLayout arm_layout = tab_.getLayout("Arm", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(0, 2);
    arm_layout.addNumber("Position (deg)", () -> Math.toDegrees(arm.getAngle()))
        .withPosition(0, 0);
    arm_layout.addNumber("Velocity (dps)", () -> Math.toDegrees(arm.getAngularVelocity()))
        .withPosition(1, 0);
    arm_layout.addNumber("Velocity Setpoint (dps)",
            () -> Math.toDegrees(arm.getAngularVelocitySetpoint()))
        .withPosition(0, 1);

    // Add extender information
    ShuffleboardLayout extender_layout = tab_.getLayout("Extender", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(2, 2);
    extender_layout.addNumber("Position (in)", () -> Units.metersToInches(extender.getPosition()))
        .withPosition(0, 0);
    extender_layout.addNumber("Velocity (ips)", () -> Units.metersToInches(extender.getVelocity()))
        .withPosition(1, 0);
    extender_layout.addNumber("Velocity Setpoint (ips)",
            () -> Units.metersToInches(extender.getVelocitySetpoint()))
        .withPosition(0, 1);
  }

  public void periodic() {
    for (Runnable fn : periodic_registry_)
      fn.run();
  }
}
