// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import org.ghrobotics.frc2023.commands.DriveBalance;
import org.ghrobotics.frc2023.commands.GoToChargeStation;
import org.ghrobotics.frc2023.commands.GoToTarget;
import org.ghrobotics.frc2023.commands.Score;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.Gyroscope;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneAndBalance extends SequentialCommandGroup {
  /** Creates a new ScoreOneAndBalance. */
  public ScoreOneAndBalance(String targetSide, String targetHeight, Drivetrain drivetrain, PoseEstimator poseEstimator, Gyroscope gyroscope) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
          // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      //new InstantCommand(() -> poseEstimator.setCurrentPose(AutoConfig.kStartToLeftOfBlueTag.getInitialPose())),
      /*new DriveTrajectory(drivetrain, poseEstimator, TrajectoryGenerator.generateTrajectory
      (poseEstimator.getCurrentPose(), List.of(), Arena.tagPositions[5].transformBy(Arena.blueTransform[0]).toPose2d(), 
      AutoConfig.kForwardConfig))*/
     // new DriveTrajectory(drivetrain, poseEstimator, auto_config_.kStartToLeftOfBlueTag)
     new GoToTarget(), 
     new WaitCommand(2),
     new Score(poseEstimator, drivetrain),
     new WaitCommand(2),
     new GoToChargeStation(),
     new WaitCommand(2),
     new DriveBalance(drivetrain, gyroscope)
    );
  }
}
