// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /**
   * Creates a new Elevator.
   */
  public Elevator() {

    //2 neos
    //1 gearbox
    //target position vs current positon
    //min and max postion --> constants
    //modes --> rest, target mode
    //target mode logic --> current pose vs target pose difference
    //upon button click, command runs and that command has mode.target mode, and also the target
    // position

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
