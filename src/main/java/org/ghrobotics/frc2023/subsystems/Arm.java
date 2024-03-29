// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.PipedOutputStream;

public class Arm extends SubsystemBase {
  // Motor Controller
  private final CANSparkMax leader_;

  // Encoder
  private final RelativeEncoder encoder_;

  // Control
  private final ArmFeedforward ff_;
  private final ProfiledPIDController fb_;
  private boolean reset_pid_ = false;

  // Simulation
  private final SingleJointedArmSim physics_sim_;
  private final SimDeviceSim leader_sim_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();
  private OutputType output_type_ = OutputType.PERCENT;


  //Constructor
  public Arm() {
    // Initialize motor controllers
    leader_ = new CANSparkMax(Constants.kLeaderId, MotorType.kBrushless);
    leader_.restoreFactoryDefaults();
    leader_.setInverted(true);
    leader_.setIdleMode(CANSparkMax.IdleMode.kCoast);

    // Initialize encoder
    encoder_ = leader_.getEncoder();
    encoder_.setPositionConversionFactor(2 * Math.PI / Constants.kGearRatio);
    encoder_.setVelocityConversionFactor(2 * Math.PI / Constants.kGearRatio / 60);

    // Initialize control
    ff_ = new ArmFeedforward(Constants.kS, Constants.kG, Constants.kV, Constants.kA);
    fb_ = new ProfiledPIDController(Constants.kP, 0, 0, new TrapezoidProfile.Constraints(
        Constants.kMaxVelocity, Constants.kMaxAcceleration));

    // Initialize simulation
    physics_sim_ = new SingleJointedArmSim(
        LinearSystemId.identifyPositionSystem(Constants.kV, Constants.kA),
        DCMotor.getNEO(1), Constants.kGearRatio, Constants.kArmLength, Constants.kMinAngle,
        Constants.kMaxAngle, false
    );
    leader_sim_ = new SimDeviceSim("SPARK MAX [" + Constants.kLeaderId + "]");

    // Safety features
    leader_.setSmartCurrentLimit((int) Constants.kCurrentLimit);
    leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.kMinAngle);
    leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.kMaxAngle);

    // Reset encoder
    zero();
    physics_sim_.setState(VecBuilder.fill(Constants.kMaxAngle, 0));
  }

  @Override
  public void periodic() {
    // Read inputs
    io_.angle = encoder_.getPosition();
    io_.angular_velocity = encoder_.getVelocity();
    io_.current = leader_.getOutputCurrent();

    if (io_.wants_zero) {
      io_.wants_zero = false;
      encoder_.setPosition(Constants.kMaxAngle);
    }

    // Reset controller if we have to
    if (reset_pid_) {
      reset_pid_ = false;
      fb_.reset(io_.angle, io_.angular_velocity);
    }

    // Write outputs
    switch (output_type_) {
      case PERCENT:
        leader_.set(io_.demand);

        // Set simulated inputs
        if (RobotBase.isSimulation())
          leader_sim_.getDouble("Applied Output").set(io_.demand * 12);

        break;
      case ANGLE:
        double feedback = fb_.calculate(io_.angle);

        double velocity_setpoint = fb_.getSetpoint().velocity;
        double acceleration_setpoint = (velocity_setpoint - io_.angular_velocity) / 0.02;
        double feedforward = ff_.calculate(io_.angle, velocity_setpoint, acceleration_setpoint);

        leader_.setVoltage(feedback + feedforward);
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // Update physics sim with inputs
    double voltage = leader_.getAppliedOutput();
    if (output_type_ == OutputType.ANGLE)
      voltage -= Constants.kG * Math.cos(io_.angle);
    physics_sim_.setInputVoltage(voltage);

    // Update physics sim forward in time
    physics_sim_.update(0.02);

    // Update encoder values
    leader_sim_.getDouble("Position").set(physics_sim_.getAngleRads());
    leader_sim_.getDouble("Velocity").set(physics_sim_.getVelocityRadPerSec());
  }

  public void setPercent(double percent) {
    output_type_ = OutputType.PERCENT;
    io_.demand = percent;
    reset_pid_ = true;
  }

  public void setAngle(double angle) {
    output_type_ = OutputType.ANGLE;
    fb_.setGoal(angle);
    reset_pid_ = true;
  }

  public void enableSoftLimits(boolean value) {
    leader_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, value);
    leader_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, value);
  }

  public void zero() {
    io_.wants_zero = true;
  }

  // Position Getter
  public double getAngle() {
    return io_.angle;
  }

  public double getAngularVelocity() {
    return io_.angular_velocity;
  }

  public double getAngularVelocitySetpoint() {
    return fb_.getSetpoint().velocity;
  }

  public double getCurrent() {
    return io_.current;
  }

  // Output Type
  private enum OutputType {
    PERCENT, ANGLE
  }

  //IO
  private static class PeriodicIO {
    //Inputs
    double angle;
    double angular_velocity;
    double current;

    //Outputs
    boolean wants_zero;
    double demand;
  }

  // Constants
  private static class Constants {
    // Motor Controller
    public static final int kLeaderId = 8;

    // Physical Constants
    public static final double kGearRatio = 49.0 * 33.0 / 15.0;
    public static final double kMinAngle = Math.toRadians(-20);
    public static final double kMaxAngle = Math.toRadians(126);
    public static final double kArmLength = 0.15;

    // Feedforward
    public static final double kA = 0.041608; //volts * seconds^2 / radians
    public static final double kG = 0.38624; //volts
    public static final double kS = 0.06490; //volts
    public static final double kV = 2.20370; //volts * seconds/radians

    // Current Limit
    public static final double kCurrentLimit = 50;

    // Control
    public static double kMaxVelocity = 3.14;
    public static double kMaxAcceleration = 3.14;
    public static double kP = 0.01;
  }
}
