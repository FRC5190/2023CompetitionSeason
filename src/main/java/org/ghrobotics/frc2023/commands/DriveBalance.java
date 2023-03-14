package org.ghrobotics.frc2023.commands;

import edu.wpi.first.math.MathUtil;
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
  private boolean begin_balance = false;

  // Constructor
  public DriveBalance(Drivetrain drivetrain) {
    // Assign member variables
    drivetrain_ = drivetrain;
    
    // Initialize PID controller
    balance_controller_ = new PIDController(Constants.kP, 0, Constants.kD);

    //Set begin_balance
    begin_balance = false;

    // Add subsystem requirements
    addRequirements(drivetrain_);
  }

  @Override
  public void initialize() {
    balance_controller_.setSetpoint(0);
  }

  @Override
  public void execute() {
    // Calculate controller output
    double output_speed = balance_controller_.calculate(drivetrain_.getPitch());

    //During testing if speed is too much, uncomment this to clamp the speed
    //output_speed = MathUtil.clamp(output_speed, -0.1, 0.1);
    
    // Set speed
    if (Math.abs(drivetrain_.getPitch()) < Math.toRadians(8)) {
      output_speed = MathUtil.clamp(output_speed, -0.05, 0.05);
    }
    drivetrain_.setVelocity(output_speed, output_speed);
        
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain_.setVelocity(0, 0);
  }

  public boolean isBalancing(){
    return (Math.abs(drivetrain_.getPitch()) > Math.toRadians(7)) ? true : false;
  }

  // Constants
  private static class Constants {
    // Ramp Angle
    public static final double kRampAngle = Math.toRadians(15.0);

    // PID Constants
    public static final double kP = AutoConfig.kBalanceRampApproachSpeed / kRampAngle;
    public static final double kD = 0.001;
  }
}
