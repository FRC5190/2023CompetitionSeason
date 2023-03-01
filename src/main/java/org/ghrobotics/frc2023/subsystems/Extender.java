// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import org.ghrobotics.frc2023.Superstructure.SuperstructureState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase {

  // Motor Controllers
  private final CANSparkMax leader_;

  // Sensors
  private final RelativeEncoder encoder_;

  // Feedforward
  private final SimpleMotorFeedforward feedforward_;

  // Feedback
  private final SparkMaxPIDController pid_controller_;

  public Extender() {

      // Initialize motor controllers
      leader_ = new CANSparkMax(Constants.kLeaderId, MotorType.kBrushless);
      leader_.setInverted(false);
      // figure out if motor should be inverted

      encoder_ = leader_.getEncoder();
      // add conversion factor based on gear ratio - needs to be fixed
      encoder_.setPositionConversionFactor(2 * Math.PI / Constants.kGearRatio);
      encoder_.setVelocityConversionFactor(2 * Math.PI / Constants.kGearRatio / 60);
      
      feedforward_ = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
      pid_controller_ = leader_.getPIDController();
      pid_controller_.setP(Constants.kP);

  }

  public double getPosition() {
      return encoder_.getPosition();
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

  public static class Constants {
      public static final int kLeaderId = 0;

      public static final double kMinPosition = 0;
      public static final double kMaxPosition = 0;
      // change based on length

      // public static final double kOutputLimit = 0;

      // Gear ratio
      public static final double kGearRatio = 12;

      // Feedforward 
      public static final double kS = 0; // volts
      public static final double kV = 0; // volts * sec / distance 
      public static final double kA = 0; // volts * sec^2 / distance

      // Feedback
      public static final double kP = 0;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
