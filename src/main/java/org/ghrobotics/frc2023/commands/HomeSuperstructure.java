package org.ghrobotics.frc2023.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2023.subsystems.Arm;
import org.ghrobotics.frc2023.subsystems.Elevator;
import org.ghrobotics.frc2023.subsystems.Extender;

public class HomeSuperstructure extends CommandBase {
  // Subsystems
  private final Elevator elevator_;
  private final Extender extender_;
  private final Arm arm_;

  // Filters
  private final MedianFilter elevator_filter_;
  private final MedianFilter extender_filter_;
  private final MedianFilter arm_filter_;

  // Reset Complete?
  private boolean elevator_zeroed_;
  private boolean extender_zeroed_;
  private boolean arm_zeroed_;

  // Timer
  private final Timer timer_;

  // Constructor
  public HomeSuperstructure(Elevator elevator, Extender extender, Arm arm) {
    // Assign member variables
    elevator_ = elevator;
    extender_ = extender;
    arm_ = arm;

    // Initialize filters
    elevator_filter_ = new MedianFilter(Constants.kFilterSize);
    extender_filter_ = new MedianFilter(Constants.kFilterSize);
    arm_filter_ = new MedianFilter(Constants.kFilterSize);

    // Initialize timer
    timer_ = new Timer();

    // Add subsystem requirements
    addRequirements(elevator_, extender_, arm_);
  }

  @Override
  public void initialize() {
    // Disable soft limits for all subsystems
    elevator_.enableSoftLimits(false);
    extender_.enableSoftLimits(false);
    arm_.enableSoftLimits(false);

    // Reset tracking (if we are not doing this for the first time)
    elevator_zeroed_ = false;
    elevator_filter_.reset();
    extender_zeroed_ = false;
    extender_filter_.reset();
    arm_zeroed_ = false;
    arm_filter_.reset();

    // Start timer
    timer_.start();
  }

  @Override
  public void execute() {
    // Check whether we should start assigning currents at this point
    boolean assign = timer_.hasElapsed(Constants.kFilterSize * 0.02);

    // Run each mechanism backwards until current threshold is reached.
    if (!elevator_zeroed_) {
      // Set speed
      elevator_.setPercent(Constants.kElevatorPct);

      // Add current to filter
      double c = elevator_filter_.calculate(elevator_.getCurrent());
      double elevator_current = assign ? c : 0;

      // Check whether we are zeroed
      if (elevator_current > Constants.kElevatorCurrentThreshold)
        elevator_zeroed_ = true;
    } else {
      elevator_.setPercent(0);
    }

    if (!extender_zeroed_) {
      // Set speed
      extender_.setPercent(Constants.kExtenderPct);

      // Add current to filter
      double c = extender_filter_.calculate(extender_.getCurrent());
      double extender_current = assign ? c : 0;

      // Check whether we are zeroed
      if (extender_current > Constants.kExtenderCurrentThreshold)
        extender_zeroed_ = true;
    } else {
      extender_.setPercent(0);
    }

    if (!arm_zeroed_) {
      // Set speed
      arm_.setPercent(Constants.kArmPct);

      // Add current to filter
      double c = arm_filter_.calculate(arm_.getCurrent());
      double arm_current = assign ? c : 0;

      // Check whether we are zeroed
      if (arm_current > Constants.kArmCurrentThreshold)
        arm_zeroed_ = true;
    } else {
      arm_.setPercent(0);
    }

    SmartDashboard.putBoolean("Elevator Zero", elevator_zeroed_);
    SmartDashboard.putBoolean("Extender Zero", extender_zeroed_);
    SmartDashboard.putBoolean("Arm Zero", arm_zeroed_);
    SmartDashboard.putNumber("Arm Current", arm_.getCurrent());
    SmartDashboard.putNumber("Extender Current", extender_.getCurrent());
    SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
  }

  @Override
  public void end(boolean interrupted) {
    // Re-enable soft limits
    elevator_.enableSoftLimits(true);
    extender_.enableSoftLimits(true);
    arm_.enableSoftLimits(true);

    // Zero encoders
    if (!interrupted) {
      elevator_.zero();
      extender_.zero();
      arm_.zero();
    }

    elevator_.setPercent(0);
    extender_.setPercent(0);
    arm_.setPercent(0);

    // Stop timer
    timer_.stop();
    timer_.reset();
  }

  @Override
  public boolean isFinished() {
    return elevator_zeroed_ && extender_zeroed_ && arm_zeroed_;
  }

  // Constants
  private static class Constants {
    // Percent Values
    public static final double kElevatorPct = -0.2;
    public static final double kExtenderPct = -0.2;
    public static final double kArmPct = 0.2;

    // Current Thresholds
    public static final double kElevatorCurrentThreshold = 12;
    public static final double kArmCurrentThreshold = 30;
    public static final double kExtenderCurrentThreshold = 12;

    // Filter Size
    public static final int kFilterSize = 25;
  }
}
