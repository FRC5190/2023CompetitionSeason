// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.Limelight;

public class ChaseTag extends CommandBase {

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
  
  //private static final int TAG_TO_CHASE = 1;
  /*private static final Transform3d TAG_TO_GOAL = new Transform3d(
          new Translation3d(2, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));*/

  private final Limelight limelight_;
  private final Drivetrain drivetrain_;
  //private final Supplier<Pose2d> poseProvider_;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  /** Creates a new ChaseTag. */
  public ChaseTag(Limelight limelight, Drivetrain drivetrain) {
    limelight_ = limelight;
    drivetrain_ = drivetrain;
    //poseProvider_ = poseProvider;

    xController.setTolerance(0.4);
    yController.setTolerance(0.4);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(drivetrain_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
