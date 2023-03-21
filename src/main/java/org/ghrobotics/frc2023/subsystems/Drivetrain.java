// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import static com.revrobotics.CANSparkMax.ControlType;

public class Drivetrain extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax left_leader_;
  private final CANSparkMax left_follower_;
  private final CANSparkMax right_leader_;
  private final CANSparkMax right_follower_;

  // Sensors
  private final RelativeEncoder left_encoder_;
  private final RelativeEncoder right_encoder_;
  private final WPI_Pigeon2 gyro_;

  private double baseline_pitch_ = Math.toRadians(-2.5927734375);

  // Control
  private final SparkMaxPIDController left_pid_controller_;
  private final SparkMaxPIDController right_pid_controller_;
  private final DifferentialDriveKinematics kinematics_;
  private final SimpleMotorFeedforward l_ff_;
  private final SimpleMotorFeedforward r_ff_;

  private double l_setpoint_ = 0;
  private double r_setpoint_ = 0;

  // Simulation
  private final DifferentialDrivetrainSim physics_sim_;
  private final SimDeviceSim left_leader_sim_;
  private final SimDeviceSim right_leader_sim_;
  private final BasePigeonSimCollection gyro_sim_;

  // IO
  private OutputType output_type_ = OutputType.PERCENT;
  private final PeriodicIO io_ = new PeriodicIO();

  // Balance Mode
  private final BooleanSupplier balance_mode_;

  // Constructor
  public Drivetrain(BooleanSupplier balance_mode) {
    // Initialize motor controllers
    left_leader_ = new CANSparkMax(Constants.kLeftLeaderId, MotorType.kBrushless);
    left_leader_.restoreFactoryDefaults();
    left_leader_.setIdleMode(IdleMode.kBrake);
    left_leader_.enableVoltageCompensation(12);
    left_leader_.setInverted(true);

    left_follower_ = new CANSparkMax(Constants.kLeftFollowerId, MotorType.kBrushless);
    left_follower_.restoreFactoryDefaults();
    left_follower_.setIdleMode(IdleMode.kBrake);
    left_follower_.enableVoltageCompensation(12);
    left_follower_.follow(left_leader_);

    right_leader_ = new CANSparkMax(Constants.kRightLeaderId, MotorType.kBrushless);
    right_leader_.restoreFactoryDefaults();
    right_leader_.setIdleMode(IdleMode.kBrake);
    right_leader_.enableVoltageCompensation(12);
    right_leader_.setInverted(false);

    right_follower_ = new CANSparkMax(Constants.kRightFollowerId, MotorType.kBrushless);
    right_follower_.restoreFactoryDefaults();
    right_follower_.setIdleMode(IdleMode.kBrake);
    right_follower_.enableVoltageCompensation(12);
    right_follower_.follow(right_leader_);

    // Initialize encoders
    left_encoder_ = left_leader_.getEncoder();
    left_encoder_.setPositionConversionFactor(
        2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio);
    left_encoder_.setVelocityConversionFactor(
        2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio / 60);

    right_encoder_ = right_leader_.getEncoder();
    right_encoder_.setPositionConversionFactor(
        2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio);
    right_encoder_.setVelocityConversionFactor(
        2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio / 60);

    // Initialize gyro
    gyro_ = new WPI_Pigeon2(Constants.kGyroId);
    gyro_.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10);

    // Initialize PID controllers.
    left_pid_controller_ = left_leader_.getPIDController();
    left_pid_controller_.setP(Constants.kLeftKp);

    right_pid_controller_ = right_leader_.getPIDController();
    right_pid_controller_.setP(Constants.kRightKp);

    // Initialize feedforward.
    l_ff_ = new SimpleMotorFeedforward(Constants.kLKs, Constants.kLKv, Constants.kLKa);
    r_ff_ = new SimpleMotorFeedforward(Constants.kRKs, Constants.kRKv, Constants.kRKa);

    // Initialize kinematics
    kinematics_ = new DifferentialDriveKinematics(Constants.kTrackWidth);

    // Initialize simulation
    physics_sim_ = new DifferentialDrivetrainSim(LinearSystemId.identifyDrivetrainSystem(
        Constants.kLinearKv, Constants.kLinearKa, Constants.kAngularKv, Constants.kAngularKa),
        DCMotor.getNEO(2),
        Constants.kGearRatio,
        Constants.kTrackWidth,
        Constants.kWheelRadius,
        null);
    left_leader_sim_ = new SimDeviceSim("SPARK MAX [" + Constants.kLeftLeaderId + "]");
    right_leader_sim_ = new SimDeviceSim("SPARK MAX [" + Constants.kRightLeaderId + "]");
    gyro_sim_ = gyro_.getSimCollection();

    // Initialize Balance Mode
    balance_mode_ = balance_mode;
  }

  @Override
  public void periodic() {
    // Read inputs
    io_.l_position = left_encoder_.getPosition();
    io_.r_position = right_encoder_.getPosition();
    io_.l_velocity = left_encoder_.getVelocity();
    io_.r_velocity = right_encoder_.getVelocity();
    io_.angle = Math.toRadians(gyro_.getYaw());
    io_.pitch = Math.toRadians(gyro_.getRoll());

    // Set output limit in balance mode
    boolean limit_output_ = balance_mode_.getAsBoolean();

    switch (output_type_) {
      case PERCENT:
        left_leader_.set(limit_output_ ?
            MathUtil.clamp(io_.l_demand, -Constants.kOutputLimit, Constants.kOutputLimit) :
            io_.l_demand);
        right_leader_.set(limit_output_ ?
            MathUtil.clamp(io_.r_demand, -Constants.kOutputLimit, Constants.kOutputLimit) :
            io_.r_demand);

        // Set simulated inputs.
        if (RobotBase.isSimulation()) {
          left_leader_sim_.getDouble("Applied Output").set(io_.l_demand * 12);
          right_leader_sim_.getDouble("Applied Output").set(io_.r_demand * 12);
        }
        break;
      case VELOCITY:
        // Calculate feedforward value and add to built-in motor controller PID.
        double l_acc = (io_.l_demand - l_setpoint_) / 0.02;
        double r_acc = (io_.r_demand - r_setpoint_) / 0.02;

        double l_volts = l_ff_.calculate(io_.l_demand, l_acc);
        double r_volts = r_ff_.calculate(io_.r_demand, r_acc);

        l_setpoint_ = io_.l_demand;
        r_setpoint_ = io_.r_demand;

        left_pid_controller_.setReference(io_.l_demand, ControlType.kVelocity, 0, l_volts);
        right_pid_controller_.setReference(io_.r_demand, ControlType.kVelocity, 0, r_volts);

        // Set simulated inputs
        if (RobotBase.isSimulation()) {
          left_leader_sim_.getDouble("Applied Output").set(l_volts);
          right_leader_sim_.getDouble("Applied Output").set(r_volts);
        }
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // Update physics sim with inputs
    physics_sim_.setInputs(left_leader_.getAppliedOutput(), right_leader_.getAppliedOutput());

    // Update physics sim forward in time
    physics_sim_.update(0.02);

    // Update encoder values
    left_leader_sim_.getDouble("Position").set(physics_sim_.getLeftPositionMeters());
    left_leader_sim_.getDouble("Velocity").set(physics_sim_.getLeftVelocityMetersPerSecond());
    right_leader_sim_.getDouble("Position").set(physics_sim_.getRightPositionMeters());
    right_leader_sim_.getDouble("Velocity").set(physics_sim_.getRightVelocityMetersPerSecond());
    gyro_sim_.setRawHeading(physics_sim_.getHeading().getRadians());
  }

  // Pitch Calibration
  public void calibratePitch() {
    baseline_pitch_ = io_.pitch;
  }

  // Percent Setter
  public void setPercent(double l, double r) {
    output_type_ = OutputType.PERCENT;
    io_.l_demand = l;
    io_.r_demand = r;
    l_setpoint_ = 0;
    r_setpoint_ = 0;
  }

  // Velocity Setter
  public void setVelocity(double l, double r) {
    output_type_ = OutputType.VELOCITY;
    io_.l_demand = l;
    io_.r_demand = r;
  }

  // Brake Mode Setter
  public void setBrakeMode(boolean value) {
    IdleMode mode = value ? IdleMode.kBrake : IdleMode.kCoast;
    left_leader_.setIdleMode(mode);
    left_follower_.setIdleMode(mode);
    right_leader_.setIdleMode(mode);
    right_follower_.setIdleMode(mode);
  }

  // Left Position Getter
  public double getLeftPosition() {
    return io_.l_position;
  }

  // Right Position Getter
  public double getRightPosition() {
    return io_.r_position;
  }

  // Left Velocity Getter
  public double getLeftVelocity() {
    return io_.l_velocity;
  }

  // Right Velocity Getter
  public double getRightVelocity() {
    return io_.r_velocity;
  }

  // Angle Getter
  public double getAngle() {
    return io_.angle;
  }

  // Pitch Getter
  public double getPitch() {
    return io_.pitch - baseline_pitch_;
  }

  // Average Velocity Getter
  public double getAverageVelocity() {
    return (io_.l_velocity + io_.r_velocity) / 2;
  }

  // Kinematics Getter
  public DifferentialDriveKinematics getKinematics() {
    return kinematics_;
  }

  enum OutputType {
    PERCENT, VELOCITY
  }

  public static class PeriodicIO {
    // Inputs
    double l_position;
    double r_position;
    double l_velocity;
    double r_velocity;
    double angle;
    double pitch;

    // Outputs
    double l_demand;
    double r_demand;
  }

  public static class Constants {
    // Motor Controller IDs
    public static final int kLeftLeaderId = 3;
    public static final int kLeftFollowerId = 4;
    public static final int kRightLeaderId = 1;
    public static final int kRightFollowerId = 2;

    // Gyro IDs
    public static final int kGyroId = 17;

    // Hardware
    public static double kGearRatio = 10.18;
    public static double kWheelRadius = 0.0762;
    public static double kTrackWidth = 0.65921;

    // Control
    public static final double kLinearKv = 2.6642;
    public static final double kLinearKa = 0.6999;
    public static final double kAngularKv = 2.9663;
    public static final double kAngularKa = 0.5675;

    public static final double kLKs = 0.1168;
    public static final double kLKv = 2.7628;
    public static final double kLKa = 0.26228;
    public static final double kRKs = 0.02646;
    public static final double kRKv = 2.6882;
    public static final double kRKa = 0.41251;

    public static final double kLeftKp = 1.0;
    public static final double kRightKp = 1.0;

    // Output Limit
    public static final double kOutputLimit = 0.4;
  }
}
