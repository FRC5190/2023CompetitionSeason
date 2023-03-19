// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.commands;

import org.ghrobotics.frc2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegrees extends CommandBase {
  /** Creates a new TurnDegrees. */

  private final Drivetrain drivetrain_;
  private final double angle_;

  public TurnDegrees(Drivetrain drivetrain, double angle) {
    drivetrain_ = drivetrain;
    angle_ = Math.toRadians(angle);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain_.setPercent(-0.5, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain_.getAngle() - angle_ < Constants.kTolerance;
  }

  public static class Constants {
    public static final double kTolerance = 2;
  }

}
