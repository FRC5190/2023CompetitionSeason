// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.Gyroscope;

public class AutoBalancing extends CommandBase {

  public final Gyroscope gyro_;
  public final Drivetrain drivetrain_;

  /**
   * Creates a new AutoBalancing.
   */
  public AutoBalancing(Gyroscope gyro, Drivetrain drivetrain) {
    gyro_ = gyro;
    drivetrain_ = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gyro_);
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
