// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {

  private final Solenoid pivot_;
  private final CANSparkMax left_leader_;
  private final CANSparkMax right_leader_;

  private final PeriodicIO io_ = new PeriodicIO();

  /** Creates a new Grabber. */
  public Grabber() {

    left_leader_ = new CANSparkMax(0, MotorType.kBrushless);
    right_leader_ = new CANSparkMax(0, MotorType.kBrushless);

    pivot_ = new Solenoid(PneumaticsModuleType.CTREPCM, 0);


    //2 pistons
    //2 neos
    //eject, intake, rest --> wheels
    //grab, release --> pistons

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static class PeriodicIO {}

  public static class Constants{}
}
