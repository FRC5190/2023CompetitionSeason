// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import org.ghrobotics.frc2023.Superstructure.SuperstructureState;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /**
   * Creates a new Arm.
   */
  public Arm() {

    leader_ = new CANSparkMax(Constants.kMotorId, MotorType.kBrushless);

    // ?????????????????
    encoder_ = new CANCoder(Constants.kEncoderId);
    encoder_.configFactoryDefault();
    encoder_.configFeedbackCoefficient(0.087890625 / Constants.kGearRatio, "deg", SensorTimeBase.PerSecond);
    // encoder_.configSensorDirection(true);
    encoder_.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_2Ms);
    encoder_.configVelocityMeasurementWindow(4);


    feedforward_ = new ArmFeedforward(Constants.kS, Constants.kG, Constants.kV, Constants.kA);
    pid_controller_ = leader_.getPIDController();
    pid_controller_.setP(Constants.kP);

  }

  public double getPosition() {
    return (encoder_.getPosition() + 180);
  }

  public void setPosition(SuperstructureState state) {
    // positive if target in front, negative if behind
    double dist = getTargetPosition(state) - getPosition();

    double v = dist / 0.02;
    double a = (v - getVelocity()) / 0.02;
    double ff = feedforward_.calculate(v, a);

    leader_.setVoltage(feedforward_.calculate(v, a));

    pid_controller_.setReference(v, ControlType.kVelocity, 0, ff);
  }

  public double getVelocity() {
      return encoder_.getVelocity();
  }

  // find values
  public double getTargetPosition(SuperstructureState state) {
      switch (state) {
          case SCORE_HIGH:
              return 0.0;
          case SCORE_MID:
              return 0.0;
          case PICKUP_GROUND:
              return 0.0;
          case PICKUP_SUBSTATION:
              return 0.0;
          default:
              return 0.0;
      }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static class Constants {
    public static final int kMotorId = 0;
    public static final int kEncoderId = 0;

    public static final double kMinPosition = 0;
    public static final double kMaxPosition = 0;
    // change based on length

    // Encoder conversions
    public static final double kGearRatio = 49;

    // Feedforward constants
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // Feedback constants
    public static final double kP = 0;

  }
}
