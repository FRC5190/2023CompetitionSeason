// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {

  private final Solenoid pivot_;
  private final CANSparkMax left_leader_;
  private final CANSparkMax right_leader_;

  private final boolean limit_output_ = false;

  private OutputType output_type_ = OutputType.PERCENT;
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Creates a new Grabber.
   */
  public Grabber() {

    left_leader_ = new CANSparkMax(Constants.kLeftLeaderId, MotorType.kBrushless);
    left_leader_.restoreFactoryDefaults();
    left_leader_.setIdleMode(IdleMode.kBrake);
    left_leader_.enableVoltageCompensation(12);
    left_leader_.setInverted(true);


    right_leader_ = new CANSparkMax(Constants.kRightLeaderId, MotorType.kBrushless);
    right_leader_.restoreFactoryDefaults();
    right_leader_.setIdleMode(IdleMode.kBrake);
    right_leader_.enableVoltageCompensation(12);
    right_leader_.setInverted(false);

    pivot_ = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.kPivotId);


    //2 pistons
    //2 neos
    //eject, intake, rest --> wheels
    //grab, release --> pistons

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (io_.wants_pneumatic_update){
      pivot_.set(io_.pivot_value);
    }



    switch (output_type_) {
      case PERCENT:
        left_leader_.set(limit_output_ ? 
          MathUtil.clamp(io_.demand, -Constants.kOutputLimit, Constants.kOutputLimit) :
          io_.demand);
        right_leader_.set(limit_output_ ? 
          MathUtil.clamp(io_.demand, -Constants.kOutputLimit, Constants.kOutputLimit) :
          io_.demand);
    }
  }

  public void setPercent(double value) {
    output_type_ = OutputType.PERCENT;
    io_.demand = value;
  }

  public void setBrakeMode(boolean value) {
    IdleMode mode = value ? IdleMode.kBrake : IdleMode.kCoast;
    left_leader_.setIdleMode(mode);
    right_leader_.setIdleMode(mode);
  }

  public void setPivot(boolean value){
    io_.wants_pneumatic_update = true;
    io_.pivot_value = value;
  }

  enum OutputType {
    PERCENT
  }

  public static class PeriodicIO {
    double demand;
    double l_supply_current;
    double r_supply_current;
    boolean pivot_value;
    boolean wants_pneumatic_update;
  }

  public static class Constants {
    public static final int kLeftLeaderId = 9;
    public static final int kRightLeaderId = 10;
    public static final int kPivotId = 0;
    public static final double kOutputLimit = 0.3;
  }
}
