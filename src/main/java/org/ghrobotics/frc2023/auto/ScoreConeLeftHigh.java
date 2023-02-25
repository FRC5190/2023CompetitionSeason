// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.util.List;
import org.ghrobotics.frc2023.Arena;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.commands.DriveTrajectory;
import org.ghrobotics.frc2023.auto.AutoConfig;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeLeftHigh extends SequentialCommandGroup {
  /** Creates a new scoreConeLeftHigh. */
  public ScoreConeLeftHigh(PoseEstimator poseEstimator, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> poseEstimator.setCurrentPose(AutoConfig.kStartToLeftOfBlueTagNV.getInitialPose())),
      /*new DriveTrajectory(drivetrain, poseEstimator, TrajectoryGenerator.generateTrajectory
      (poseEstimator.getCurrentPose(), List.of(), Arena.tagPositions[5].transformBy(Arena.blueTransform[0]).toPose2d(), 
      AutoConfig.kForwardConfig))*/

      new DriveTrajectory(drivetrain, poseEstimator, AutoConfig.kStartToLeftOfBlueTagNV)
    );
  }
}
