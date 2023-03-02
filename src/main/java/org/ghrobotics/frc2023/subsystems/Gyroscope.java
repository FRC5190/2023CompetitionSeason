// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyroscope extends SubsystemBase {
  // Gyro
  private final WPI_Pigeon2 gyro_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();

  // Constructor
  public Gyroscope() {
    // Initialize gyro
    gyro_ = new WPI_Pigeon2(Constants.kGyroId);
  }

  @Override
  public void periodic() {
    // Read inputs
    io_.pitch = Math.toRadians(gyro_.getRoll());
    io_.angle = gyro_.getRotation2d();
  }

  // Angle Getter
  public Rotation2d getAngle() {
    return io_.angle;
  }

  // Pitch Getter
  public double getPitch() {
    return io_.pitch;
  }

  private static class PeriodicIO {
    // Inputs
    Rotation2d angle = new Rotation2d();
    double pitch;
  }
  
  // Constants class
  public static class Constants {
    public static final int kGyroId = 17;
  }
}
