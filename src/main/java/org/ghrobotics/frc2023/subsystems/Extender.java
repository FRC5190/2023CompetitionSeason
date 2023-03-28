// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
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
  private boolean reset_pid_ = true;

  // Simulation
  private final ElevatorSim physics_sim_;
  private final SimDeviceSim leader_sim_;

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

    // Initialize simulation
    physics_sim_ = new ElevatorSim(
        LinearSystemId.identifyPositionSystem(
            Constants.kV, Constants.kA), DCMotor.getNEO(1),
        Constants.kGearRatio, Constants.kSprocketDiameter / 2, Constants.kMinLength,
        Constants.kMaxLength, false);
    leader_sim_ = new SimDeviceSim("SPARK MAX [" + Constants.kLeaderId + "]");

    // Safety features
    leader_.setSmartCurrentLimit((int) Constants.kCurrentLimit);
    leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.kMinLength);
    leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.kMaxLength);
    enableSoftLimits(true);

    // Reset encoder position
    zero();
  }

  @Override
  public void periodic() {
    // Read inputs
    io_.position = encoder_.getPosition();
    io_.velocity = encoder_.getVelocity();
    io_.current = leader_.getOutputCurrent();

    if (io_.wants_zero) {
      io_.wants_zero = false;
      encoder_.setPosition(0);
    }

    // Reset controller if we have to
    if (reset_pid_) {
      reset_pid_ = false;
      fb_.reset(io_.position, io_.velocity);
    }

    // Write outputs
    switch (output_type_) {
      case PERCENT:
        leader_.set(io_.demand);

        // Set simulated inputs
        if (RobotBase.isSimulation())
          leader_sim_.getDouble("Applied Output").set(io_.demand * 12);

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

  @Override
  public void simulationPeriodic() {
    // Update physics sim with inputs
    physics_sim_.setInputVoltage(leader_.getAppliedOutput());

    // Update physics sim forward in time
    physics_sim_.update(0.02);

    // Update encoder values
    leader_sim_.getDouble("Position").set(physics_sim_.getPositionMeters());
    leader_sim_.getDouble("Velocity").set(physics_sim_.getVelocityMetersPerSecond());
  }

  public void resetFeedback() {
    reset_pid_ = true;
  }

  public void setPercent(double percent) {
    output_type_ = OutputType.PERCENT;
    io_.demand = percent;
  }

  public void setPosition(double position) {
    output_type_ = OutputType.POSITION;
    io_.demand = position;
  }

  public void enableSoftLimits(boolean value) {
    leader_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, value);
    leader_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, value);
  }

  public void zero() {
    io_.wants_zero = true;
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

  public double getCurrent() {
    return io_.current;
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
    double current;

    // Outputs
    boolean wants_zero;
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
    public static final double kS = 0.090508;
    public static final double kV = 10.654;
    public static final double kA = 0.1604;

    // Current Limit
    public static final double kCurrentLimit = 50;

    // Control
    public static double kMaxVelocity = 0.6;
    public static double kMaxAcceleration = 0.4;
    public static double kP = 0.003;
  }
}
