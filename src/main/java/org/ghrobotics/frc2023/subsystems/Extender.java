// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class Extender extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax leader_;

  // Encoder
  private final RelativeEncoder encoder_;

  // Control
  private final ProfiledPIDController fb_;
  private final SimpleMotorFeedforward ff_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();
  private OutputType output_type_ = OutputType.PERCENT;


  public Extender() {
    // Initialize motor controllers
    leader_ = new CANSparkMax(Constants.kLeaderId, kBrushless);
    leader_.restoreFactoryDefaults();
    leader_.setInverted(true);
    leader_.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Initialize encoder
    encoder_ = leader_.getEncoder();
    encoder_.setPositionConversionFactor(
        Math.PI * Constants.kSprocketDiameter / Constants.kGearRatio);
    encoder_.setVelocityConversionFactor(
        Math.PI * Constants.kSprocketDiameter / Constants.kGearRatio / 60);

    // Initialize control
    ff_ = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
    fb_ = new ProfiledPIDController(Constants.kP, 0, 0, new TrapezoidProfile.Constraints(
        Constants.kMaxVelocity, Constants.kMaxAcceleration));

    // Safety features
    leader_.setSmartCurrentLimit((int) Constants.kCurrentLimit);
    leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.kMinLength);
    leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.kMaxLength);

    encoder_.setPosition(0);
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

  // Position Getter
  public double getPosition() {
    return io_.position;
  }

  public double getVelocity() {
    return io_.velocity;
  }

  public double getVelocitySetpoint() {
    return fb_.getSetpoint().velocity;
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
    public static final int kLeaderId = 7;

    // Physical Constants
    public static final double kGearRatio = 12.0;
    public static final double kSprocketDiameter = 0.045466;
    public static final double kMinLength = 0.0;
    public static final double kMaxLength = Units.inchesToMeters(10);

    // Feedforward
    public static final double kS = 0.086;
    public static final double kV = 10.705;
    public static final double kA = 0.23276;

    // Current Limit
    public static final double kCurrentLimit = 50;

    // Control
    public static double kMaxVelocity = 0.5;
    public static double kMaxAcceleration = 0.6;
    public static double kP = 0.03;
  }
}
