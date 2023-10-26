// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.ghrobotics.frc2023.subsystems.Drivetrain;


/**
 * Add your docs here.
 */
public class DriveTeleop extends CommandBase {
  private final Drivetrain drivetrain_;
  private final CommandXboxController controller_;

  public DriveTeleop(Drivetrain drivetrain, CommandXboxController controller) {
    drivetrain_ = drivetrain;
    controller_ = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain_);
  }

  @Override
  public void execute() {
    double forward = -controller_.getLeftY * Constants.sensitivity();
    double curvature = -controller_.getLeftX * Constants.sensitivity();
    boolean quick_turn = controller_.x().getAsBoolean();

    DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(
        forward, curvature, quick_turn);

    drivetrain_.setPercent(speeds.left, speeds.right);
  }

  public static class Constants {
    public static final double sensitivity = 0.2;
  }
}
