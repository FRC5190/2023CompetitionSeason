// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  //Motor Controller
  private final CANSparkMax leader_;

  //Encoder
  private final RelativeEncoder encoder_;

  //Control
  private final ArmFeedforward ff_;
  private final ProfiledPIDController fb_;

  //IO
  private final PeriodicIO io_ = new PeriodicIO();
  private OutputType output_type_ = OutputType.PERCENT;


  //Constructor
  public Arm() {
    //Initialize motor controllers
    leader_ = new CANSparkMax(Constants.kLeaderId, MotorType.kBrushless);
    leader_.restoreFactoryDefaults();

    //Initialize encoder
    encoder_ = leader_.getEncoder();
    encoder_.setPositionConversionFactor(2 * Math.PI / Constants.kGearRatio);
    encoder_.setVelocityConversionFactor(2 * Math.PI / Constants.kGearRatio / 60);

    //Initialize control
    ff_ = new ArmFeedforward(Constants.kS, Constants.kG, Constants.kV, Constants.kA);
    fb_ = new ProfiledPIDController(Constants.kP, 0, 0, new TrapezoidProfile.Constraints(
        Constants.kMaxVelocity, Constants.kMaxAcceleration));

    //Safety features
    leader_.setSmartCurrentLimit((int) Constants.kCurrentLimit);
    leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.kMinAngle);
    leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.kMaxAngle);
  }

  @Override
  public void periodic() {
    //Read inputs
    io_.angle = encoder_.getPosition();
    io_.angular_velocity = encoder_.getVelocity();

    //Write outputs
    switch (output_type_) {
      case PERCENT:
        //leader_.set(io_.demand);
        break;
      case ANGLE:
        fb_.setGoal(io_.demand);
        double feedback = fb_.calculate(io_.angle);

        double velocity_setpoint = fb_.getSetpoint().velocity;
        double acceleration_setpoint = (velocity_setpoint - io_.angular_velocity) / 0.02;
        double feedforward = ff_.calculate(velocity_setpoint, acceleration_setpoint);

        //leader_.setVoltage(feedback + feedforward);
        break;
    }
  }

  public void setPercent(double percent) {
    output_type_ = OutputType.PERCENT;
    io_.demand = percent;
  }

  public void setAngle(double angle) {
    output_type_ = OutputType.ANGLE;
    io_.demand = angle;
  }

  //Output Type
  private enum OutputType {
    PERCENT, ANGLE
  }

  //IO
  private static class PeriodicIO {
    //Inputs
    double angle;
    double angular_velocity;

    //Outputs
    double demand;
  }

  //Constants
  private static class Constants {
    //Motor Controller
    public static final int kLeaderId = 8;

    //Physical Constants
    public static final double kGearRatio = 49.0 * 33.0 / 15.0;
    public static final double kMinAngle = Math.toRadians(-20);
    public static final double kMaxAngle = Math.toRadians(126);

    //Feedforward
    public static final double kA = 0.0; //volts * seconds^2 / radians
    public static final double kG = 0.0; //volts
    public static final double kS = 0.0; //volts
    public static final double kV = 0.0; //volts * seconds/radians

    //Current Limit
    public static final double kCurrentLimit = 50;

    //Control
    public static double kMaxVelocity = 0.3;
    public static double kMaxAcceleration = 0.3;
    public static double kP = 0.0;
  }
}
