// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class Elevator extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax leader_;
  private final CANSparkMax follower_;

  // Encoder
  private final RelativeEncoder encoder_;

  // Control
  private final ProfiledPIDController fb_;
  private final ElevatorFeedforward ff_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();
  private OutputType output_type_ = OutputType.PERCENT;


  public Elevator() {
    // Initialize motor controllers
    leader_ = new CANSparkMax(Constants.kLeaderId, kBrushless);
    leader_.restoreFactoryDefaults();

    follower_ = new CANSparkMax(Constants.kFollowerId, kBrushless);
    follower_.restoreFactoryDefaults();
    follower_.follow(leader_);

    // Initialize encoder
    encoder_ = leader_.getEncoder();
    encoder_.setPositionConversionFactor(
        Math.PI * Constants.kSprocketDiameter / Constants.kGearRatio);
    encoder_.setVelocityConversionFactor(
        Math.PI * Constants.kSprocketDiameter / Constants.kGearRatio / 60);

    // Initialize control
    ff_ = new ElevatorFeedforward(Constants.kS, Constants.kG, Constants.kV, Constants.kA);
    fb_ = new ProfiledPIDController(Constants.kP, 0, 0, new TrapezoidProfile.Constraints(
        Constants.kMaxVelocity, Constants.kMaxAcceleration));

    // Safety features
    leader_.setSmartCurrentLimit((int) Constants.kCurrentLimit);
    leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.kMinHeight);
    leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.kMaxHeight);
  }

  @Override
  public void periodic() {
    // Read inputs
    io_.position = encoder_.getPosition();
    io_.velocity = encoder_.getVelocity();

    // Write outputs
    switch (output_type_) {
      case PERCENT:
        leader_.set(io_.demand);
        break;
      case POSITION:
        fb_.setGoal(io_.demand);
        double feedback = fb_.calculate(io_.position);

        double velocity_setpoint = fb_.getSetpoint().velocity;
        double acceleration_setpoint = (velocity_setpoint - io_.velocity) / 0.02;
        double feedforward = ff_.calculate(velocity_setpoint, acceleration_setpoint);

        leader_.setVoltage(feedback + feedforward);
        break;
    }
  }

  public void setPercent(double percent) {
    output_type_ = OutputType.PERCENT;
    io_.demand = percent;
  }

  public void setPosition(double position) {
    output_type_ = OutputType.POSITION;
    io_.demand = position;
  }

  // Output Type
  private enum OutputType {
    PERCENT, POSITION
  }

  // IO
  private static class PeriodicIO {
    // Inputs
    double position;
    double velocity;

    // Outputs
    double demand;
  }

  // Constants
  private static class Constants {
    // Motor Controllers
    public static final int kLeaderId = 5;
    public static final int kFollowerId = 6;

    // Physical Constants
    public static final double kGearRatio = 20.0;
    public static final double kSprocketDiameter = 0.045;
    public static final double kMinHeight = 0.0;
    public static final double kMaxHeight = Units.inchesToMeters(40);

    // Feedforward
    public static final double kG = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    // Current Limit
    public static final double kCurrentLimit = 50;

    // Control
    public static double kMaxVelocity = 0.3;
    public static double kMaxAcceleration = 0.3;
    public static double kP = 0.0;
  }
}
