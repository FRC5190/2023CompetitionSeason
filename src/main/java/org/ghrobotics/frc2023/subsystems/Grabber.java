// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax left_leader_;
  private final CANSparkMax right_leader_;

  // Pneumatics
  private final Solenoid pivot_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();

  // Constructor
  public Grabber() {
    // Initialize motor controllers
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

    // Initialize pneumatics
    pivot_ = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.kPivotId);
  }

  @Override
  public void periodic() {
    // Read inputs
    io_.l_supply_current = left_leader_.getOutputCurrent();
    io_.r_supply_current = right_leader_.getOutputCurrent();

    // Write outputs
    if (io_.wants_pneumatic_update) {
      pivot_.set(io_.pivot_value);
      io_.wants_pneumatic_update = false;
    }

    left_leader_.set(io_.demand);
    right_leader_.set(io_.demand);
  }

  // Percent Setter
  public void setPercent(double value) {
    io_.demand = value;
  }

  // Pivot Setter
  public void setPivot(boolean value) {
    io_.wants_pneumatic_update = true;
    io_.pivot_value = value;
  }

  // Brake Mode Setter
  public void setBrakeMode(boolean value) {
    IdleMode mode = value ? IdleMode.kBrake : IdleMode.kCoast;
    left_leader_.setIdleMode(mode);
    right_leader_.setIdleMode(mode);
  }

  // IO
  public static class PeriodicIO {
    // Inputs
    double l_supply_current;
    double r_supply_current;

    // Outputs
    double demand;
    boolean pivot_value;
    boolean wants_pneumatic_update;
  }

  // Constants
  public static class Constants {
    // Motor Controllers
    public static final int kLeftLeaderId = 9;
    public static final int kRightLeaderId = 10;

    // Pneumatics
    public static final int kPivotId = 0;
  }
}
