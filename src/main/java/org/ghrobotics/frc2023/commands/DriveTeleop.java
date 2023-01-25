// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.ghrobotics.frc2023.subsystems.Drivetrain;


/** Add your docs here. */
public class DriveTeleop extends CommandBase{
    private final Drivetrain drivetrain_;
    private final XboxController controller_;

    public DriveTeleop(Drivetrain drivetrain, XboxController controller) {
        drivetrain_ = drivetrain;
        controller_ = controller;

        addRequirements(drivetrain_);
        System.out.println("in DriveTeleop Constructor");
    }

    @Override
    public void execute(){
        double forward = -controller_.getLeftY();
        double curvature = controller_.getLeftX();
        boolean quick_turn = controller_.getXButton();

        DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(
            forward, curvature, quick_turn);

        drivetrain_.setPercent(speeds.left, speeds.right);
    }

}
