// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import org.ghrobotics.frc2023.Superstructure;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Score High Cube Backwards
// * go on top of charge station and exit community
// * pick up cone/cube
// * move backwards
// * run balance command

public class ScoreCoopertitionExitBalanceBackwards extends SequentialCommandGroup {
  /** Creates a new ScoreCoopertitionExitBalanceBackwards. */
  public ScoreCoopertitionExitBalanceBackwards(Drivetrain drivetrain, Superstructure superstructure,
  PoseEstimator pose_estimator, DriverStation.Alliance alliance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
