package org.ghrobotics.frc2023.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.Gyroscope;

public class DriveBalance extends CommandBase {
  // Subsystems
  private final Drivetrain drivetrain_;
  private final Gyroscope gyroscope_;

  // Decision Variables
  private double baseline_pitch_ = 0.0;
  private boolean on_platform_ = false;
  private boolean is_balanced_ = false;

  // Constructor
  public DriveBalance(Drivetrain drivetrain, Gyroscope gyroscope) {
    // Assign member variables
    drivetrain_ = drivetrain;
    gyroscope_ = gyroscope;

    // Add subsystem requirements
    addRequirements(drivetrain_);
  }

  @Override
  public void initialize() {
    // Establish baseline pitch value when robot is flat on the floor
    baseline_pitch_ = gyroscope_.getPitch();
  }

  @Override
  public void execute() {
    // Based on whether we are on the platform or not, set speed
    double speed = on_platform_ ? Constants.kInclineSpeed : Constants.kFlatSpeed;
    drivetrain_.setVelocity(speed, speed);

    // If we weren't on the platform before, check whether we are now
    if (!on_platform_ && (gyroscope_.getPitch() - baseline_pitch_) > Constants.kOnPlatformThreshold)
      on_platform_ = true;

    // If we are on the platform and weren't balanced, check whether we are now
    if (on_platform_ && !is_balanced_ && (gyroscope_.getPitch() - baseline_pitch_) < Constants.kBalanceThreshold)
      is_balanced_ = true;

    // Telemetry: delete later
    SmartDashboard.putBoolean("on_platform", on_platform_);
    SmartDashboard.putBoolean("is_balanced", is_balanced_);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain_.setVelocity(0, 0);
  }

  @Override
  public boolean isFinished() {
    return is_balanced_;
  }

  // Constants
  private static class Constants {
    // Angles
    public static final double kOnPlatformThreshold = Math.toRadians(20);
    public static final double kBalanceThreshold = Math.toRadians(15);

    // Speeds
    public static final double kFlatSpeed = 0.2;
    public static final double kInclineSpeed = 0.05;
  }
}
