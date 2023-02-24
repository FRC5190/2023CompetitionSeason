// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

import org.ghrobotics.frc2023.Limelight;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.Gyroscope;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase {

  private final Limelight limelight_;
  private final Drivetrain drivetrain_;
  private final Gyroscope gyro_;
  private boolean tracking_target_ = false;
  private final LinearFilter alive_filter_;
  private boolean is_alive_ = false;

  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  double[] camPose = new double[6];
  double camX;
  double camY;
  double camZ;

  double[] bluePose = new double[6];
  double blueX;
  double blueY;
  double blueZ;
  double blueXR;
  double blueYR;
  double blueZR;

  double[] idValue = new double[6];
  double id;

  private final DifferentialDrivePoseEstimator poseEstimator;

  /** Creates a new PoseEstimator. */
  public PoseEstimator(Limelight limelight, Drivetrain drivetrain, Gyroscope gyroscope) {
    limelight_ = limelight;
    drivetrain_ = drivetrain;
    gyro_ = gyroscope;

    alive_filter_ = LinearFilter.movingAverage(10);

    poseEstimator =  new DifferentialDrivePoseEstimator(
        drivetrain_.kinematics_,
        new Rotation2d(0), //CHANGE
        drivetrain_.getLeftPosition(),
        drivetrain_.getRightPosition(),
        new Pose2d(0,0,new Rotation2d(0)),
        stateStdDevs, visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limelight_.periodic();

    double latency = limelight_.getLatency();
    double captureLatency = limelight_.getCaptureLatency();
    is_alive_ = alive_filter_.calculate(latency) > 11;

    tracking_target_ = limelight_.hasTarget();
    
    if (tracking_target_) {
      double timestamp = Timer.getFPGATimestamp() - (latency / 1000)  - (captureLatency / 1000);
      bluePose = limelight_.getBlueBotPose();
      blueX = bluePose[0];
      blueY = bluePose[1];
      blueZ = bluePose[2];
      blueXR = bluePose[3];
      blueYR = bluePose[4];
      blueZR = bluePose[5];

      idValue = limelight_.getID();
      id = idValue[5];

      Pose2d botPose = new Pose2d(blueX, blueY, new Rotation2d(blueXR, blueYR));
      poseEstimator.addVisionMeasurement(botPose, timestamp);
      // Transform3d camToTarget = target.getBestCameraToTarget();
      // Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
    }
    poseEstimator.update(gyro_.getGyroRotation(), 
      drivetrain_.getLeftPosition(), 
      drivetrain_.getRightPosition());
    
  }

  public Pose2d getCurrentPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public double getCamX(){
    return camX;
  }

  public double getCamY(){
    return camY;
  }

  public double getIDValue(){
    return id;
  }

  public void setCurrentPose(Pose2d newPose){
    poseEstimator.resetPosition(gyro_.getGyroRotation(), drivetrain_.getLeftPosition(), 
      drivetrain_.getRightPosition(), newPose);
  }
}
