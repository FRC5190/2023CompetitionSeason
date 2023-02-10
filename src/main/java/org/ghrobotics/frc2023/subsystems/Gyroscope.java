// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class Gyroscope extends SubsystemBase {

  private final WPI_Pigeon2 gyro_;

  private final PeriodicIO io_ = new PeriodicIO();

  /** Creates a new Gyroscope. */
  public Gyroscope() {
    gyro_ = new WPI_Pigeon2(0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io_.pitch = gyro_.getPitch();
    io_.roll = gyro_.getRoll();
    io_.yaw = gyro_.getYaw();
    io_.angle = gyro_.getRotation2d();
  }

  public Rotation2d getGyroRotation() {
    return io_.angle;
  }

  public static class PeriodicIO {
    double pitch;
    double yaw;
    double roll;
    Rotation2d angle;

  }


}