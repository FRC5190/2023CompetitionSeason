// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import org.ghrobotics.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

public class Gyroscope extends SubsystemBase {

  private final WPI_Pigeon2 gyro_;

  private final PeriodicIO io_ = new PeriodicIO();

  public boolean onRamp = false;

  // Assigns variables and calls the _pigeon object
  // PigeonIMU _pigeon = new PigeonIMU(17);
  Drivetrain drivetrain = new Drivetrain();
  int _loopCount = 0;
  double rollValue;
  double motorOutput;
  boolean finished = false;
  // If it is within 8 degree it should stop
  double slackAngle = 8;
  
  boolean onPlatform = false;

  /** Creates a new Gyroscope. */
  public Gyroscope() {
    gyro_ = new WPI_Pigeon2(17);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io_.pitch = gyro_.getPitch();
    io_.roll = gyro_.getRoll();
    io_.yaw = gyro_.getYaw();
    io_.angle = gyro_.getRotation2d();
    double[] ypr = new double[3];
    gyro_.getYawPitchRoll(ypr);
    double offset = 0.175;
    rollValue = ypr[2] + offset;

    if(rollValue > 8){
      onPlatform = true;
      System.out.println("On Platform");
    }

    if (onPlatform == true){
      teleopGyroscope();
      System.out.println("Gryoscope code Running...");
    }

    // public boolean onRamp = false;


    // Thought of using drivetrain.setBrakeMode(true) after the robot is on the platform and under 3 degree angle for more than 2 seconds
    // it will set the brake?

    if (rollValue < 3 && rollValue > -3){
      // 2 second = 2000 milliseconds 2000 milliseconds /20 milliseconds at which periodic = 100 times ;)
      for (int i = 0; i < 100; i++){
        drivetrain.setBrakeMode(true);
        System.out.println("Brake Mode On");
      }
    }

  }

  public void teleopGyroscope(){
    System.out.println("The angle is: " + rollValue);
    if((rollValue < -slackAngle) && !finished)
    {
      // Don't need to turn negative angle positive for sin function
      // rollValue = rollValue * -1;
      // motorOutput = (Math.pow(1.007, rollValue) - 1);
      rollValue = Math.toRadians(rollValue);
      motorOutput = Math.sin(rollValue)/3;
      // motorOutput = motorOutput * -1;
      System.out.println("Motor Output should be: " + (motorOutput) + "%");
    }
    else if ((rollValue > slackAngle) && !finished)
    {
      // motorOutput = (Math.pow(1.007, rollValue) - 1);
      rollValue = Math.toRadians(rollValue);
      motorOutput = Math.sin(rollValue)/3;
      System.out.println("Motor Output should be: " + (motorOutput) + "%");
    } else{
      finished = true;
      drivetrain.setPercent(0, 0);
      System.out.println("Speed is zero");
    }
    // Main "speed setter"
    drivetrain.setPercent(motorOutput, motorOutput);
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
