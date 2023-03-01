// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.List;
import org.ghrobotics.frc2023.auto.AutoConfig;
import org.ghrobotics.frc2023.auto.ScoreOne;
import org.ghrobotics.frc2023.auto.ScoreOneAndBalance;
import org.ghrobotics.frc2023.commands.DriveBalance;
import org.ghrobotics.frc2023.commands.DriveTeleop;
import org.ghrobotics.frc2023.commands.GoToTarget;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.Gyroscope;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final Drivetrain drivetrain_ = new Drivetrain();
  private final XboxController driver_controller_ = new XboxController(0);
  private final Limelight limelight_ = new Limelight("limelight");
  private final Gyroscope gyro_ = new Gyroscope();
  private final Arena arena_ = new Arena();
  private final PoseEstimator pose_estimator_ = new PoseEstimator(limelight_, drivetrain_, gyro_);
  private final SendableChooser<String> auto_selector_side = new SendableChooser<>();
  private final SendableChooser<String> auto_selector_height = new SendableChooser<>();
  private final SendableChooser<String> auto_balance = new SendableChooser<>();
  private final Field2d field_ = new Field2d();
  private String target_side_ = "Left";
  private String target_height_ = "High";
  private String auto_balance_choice = "Yes";
  private Command score_and_balance_;
  private Command score_one_;
  private final Timer timer_ = new Timer();
  public Trajectory trajectory_;
  public Pose2d start_pose;

  private final Telemetry telemetry_ = new Telemetry(drivetrain_, pose_estimator_, limelight_,
      auto_selector_side, auto_selector_height, auto_balance);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    drivetrain_.setDefaultCommand(new DriveTeleop(drivetrain_, driver_controller_));
    setUpAuto();

    pose_estimator_.setCurrentPose(new Pose2d(2, 1, Rotation2d.fromDegrees(180)));
    start_pose = pose_estimator_.getCurrentPose();

    trajectory_ =
        TrajectoryGenerator.generateTrajectory(start_pose,
            List.of(/*new Translation2d(2, 1), new Translation2d(2, -1)*/),
            new Pose2d(1, 0, Rotation2d.fromDegrees(180)),
            /*new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0))*/
            AutoConfig.kForwardConfig);

    // Create and push Field2d to SmartDashboard.
    SmartDashboard.putData(field_);

    // Push the trajectory to Field2d.
    field_.getObject("traj").setTrajectory(new GoToTarget(pose_estimator_, drivetrain_, gyro_,
        auto_balance_choice, limelight_, arena_).getTrajectory());
  }

  @Override
  public void robotPeriodic() {
    //Starts the Command Scheduler to make sure periodic() and such work.
    CommandScheduler.getInstance().run();

    telemetry_.periodic();
    limelight_.periodic();

    //new Trigger(driver_controller_::getBButton).onTrue(new ScoreConeLeftHigh(pose_estimator_,
    // drivetrain_));
    SmartDashboard.putNumber("Velocity", drivetrain_.getVelocity());
  }

  @Override
  public void autonomousInit() {
    target_side_ = auto_selector_side.getSelected();
    target_height_ = auto_selector_height.getSelected();
    auto_balance_choice = auto_balance.getSelected();

    if (target_side_ != "" && target_height_ != "" && auto_balance_choice == "Yes") {
      score_and_balance_ = new ScoreOneAndBalance(target_side_, target_height_, drivetrain_,
          pose_estimator_, gyro_, limelight_, arena_);
      score_and_balance_.schedule();
    } else if (target_side_ != "" && target_height_ != "" && auto_balance_choice == "No") {
      score_one_ = new ScoreOne(drivetrain_, pose_estimator_, gyro_, target_side_, limelight_,
          arena_);
      score_one_.schedule();
    } else if (target_side_ == "" && target_height_ == "" && auto_balance_choice == "Yes") {
      new DriveBalance(drivetrain_, gyro_).schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    timer_.start();
  }

  @Override
  public void teleopPeriodic() {
    /*double t = timer_.get();
    if (t <= 0.5){
      drivetrain_.setVelocity(0.1, 0.1);
    }
    timer_.stop();*/

  }

  @Override
  public void disabledInit() {
    timer_.start();
  }

  @Override
  public void disabledPeriodic() {
    double t = timer_.get();
    if (t <= 0.5) {
      drivetrain_.setBrakeMode(false);
    }
    timer_.stop();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private void setUpAuto() {
    //public static int ID;
    /*if (limelight_.hasTarget()){
      ID = limelight_.getID();
    }*/
    auto_selector_side.addOption("Left", "Left");
    auto_selector_side.addOption("Center", "Center");
    auto_selector_side.addOption("Right", "Right");

    auto_selector_height.addOption("High", "High");
    auto_selector_height.addOption("Mid", "Mid");

    auto_balance.addOption("Yes", "Yes");
    auto_balance.addOption("No", "No");
  }
}
