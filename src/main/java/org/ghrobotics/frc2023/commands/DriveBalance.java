package org.ghrobotics.frc2023.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2023.auto.AutoConfig;
import org.ghrobotics.frc2023.subsystems.Drivetrain;

public class DriveBalance extends CommandBase {
  // Subsystems
  private final Drivetrain drivetrain_;

  // PID Controller
  private final PIDController balance_controller_;

  // Constructor
  public DriveBalance(Drivetrain drivetrain) {
    // Assign member variables
    drivetrain_ = drivetrain;

    // Initialize PID controller
    balance_controller_ = new PIDController(Constants.kP, 0, 0);

    // Add subsystem requirements
    addRequirements(drivetrain_);
  }

  @Override
  public void execute() {
    // Calculate controller output
    double output_speed = balance_controller_.calculate(drivetrain_.getPitch());

    // Set speed
    drivetrain_.setVelocity(-output_speed, -output_speed);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain_.setVelocity(0, 0);
  }

  // Constants
  private static class Constants {
    // Ramp Angle
    public static final double kRampAngle = Math.toRadians(15.0);

    // PID Constants
    public static final double kP = AutoConfig.kBalanceRampApproachSpeed / kRampAngle;
  }
}
