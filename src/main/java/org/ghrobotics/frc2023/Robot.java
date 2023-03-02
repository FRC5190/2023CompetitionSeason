// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.ghrobotics.frc2023.subsystems.Arm;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.Elevator;
import org.ghrobotics.frc2023.subsystems.Extender;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;
import org.ghrobotics.frc2023.subsystems.Gyroscope;
import org.ghrobotics.frc2023.commands.DriveTeleop;
import org.ghrobotics.frc2023.commands.DriveBalance;
import org.ghrobotics.frc2023.Telemetry;
import org.ghrobotics.frc2023.Superstructure.SuperstructureState;
import org.ghrobotics.frc2023.Limelight;
import org.ghrobotics.frc2023.auto.ScoreOne;
import org.ghrobotics.frc2023.auto.ScoreOneAndBalance;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final Drivetrain drivetrain_ = new Drivetrain();
  private final XboxController driver_controller_ = new XboxController(0);
  private final XboxController operator_controller = new XboxController(0);  // which port?
  private final Limelight limelight_ = new Limelight("limelight");
  private final Gyroscope gyro_ = new Gyroscope();
  private final Arena arena_ = new Arena();
  private final Elevator elevator_ = new Elevator();
  private final Extender extender_ = new Extender();
  private final Arm arm_ = new Arm();
  private final Superstructure superstructure_ = new Superstructure(elevator_, 
          extender_, arm_, operator_controller, SuperstructureState.RESET);
  private final PoseEstimator pose_estimator_ = new PoseEstimator(limelight_, drivetrain_, gyro_);
  private final SendableChooser<String> auto_selector_side = new SendableChooser<>();
  private final SendableChooser<String> auto_selector_height = new SendableChooser<>();
  private final SendableChooser<String> auto_balance = new SendableChooser<>();
  private String target_side_ = "Left";
  private String target_height_ = "High";
  private String auto_balance_choice = "Yes";
  private Command score_and_balance_;
  private Command score_one_;
  private final Timer timer_ = new Timer();

  private final Telemetry telemetry_ = new Telemetry(drivetrain_, pose_estimator_, limelight_, auto_selector_side, auto_selector_height, auto_balance);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  @Override
  public void robotInit() {
    drivetrain_.setDefaultCommand(new DriveTeleop(drivetrain_, driver_controller_));
    setUpAuto();
    superstructure_.reset();
  }

  @Override
  public void robotPeriodic() {
    //Starts the Command Scheduler to make sure periodic() and such work.
    CommandScheduler.getInstance().run();

    telemetry_.periodic();
    limelight_.periodic();
    superstructure_.periodic();

    //new Trigger(driver_controller_::getBButton).onTrue(new ScoreConeLeftHigh(pose_estimator_,drivetrain_));
    SmartDashboard.putNumber("Velocity", drivetrain_.getVelocity());
  }

  @Override
  public void autonomousInit() {
   target_side_ = auto_selector_side.getSelected();
   target_height_ = auto_selector_height.getSelected();
   auto_balance_choice = auto_balance.getSelected();
  
   if (target_side_ != "" && target_height_ != "" && auto_balance_choice == "Yes") {
    score_and_balance_ = new ScoreOneAndBalance(target_side_, target_height_, drivetrain_, pose_estimator_, gyro_, limelight_, arena_)
    score_and_balance_.schedule();
   } else if (target_side_ != "" && target_height_ != "" && auto_balance_choice == "No"){
    score_one_ = new ScoreOne(drivetrain_, pose_estimator_, gyro_, target_side_, limelight_, arena_);
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

  private void setUpAuto(){
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
