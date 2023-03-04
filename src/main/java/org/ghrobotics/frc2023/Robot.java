// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.ghrobotics.frc2023.auto.AutoSelector;
import org.ghrobotics.frc2023.commands.DriveBrakeMode;
import org.ghrobotics.frc2023.commands.DriveTeleop;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.Grabber;
import org.ghrobotics.frc2023.subsystems.LED;
import org.ghrobotics.frc2023.subsystems.Limelight;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;
import org.ghrobotics.frc2023.subsystems.LED.StandardLEDOutput;
import org.ghrobotics.frc2023.commands.DriveBalance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Subsystems
  private final Drivetrain drivetrain_ = new Drivetrain();
  private final Limelight limelight_ = new Limelight("limelight");
  private final Grabber grabber_ = new Grabber();
  private final PoseEstimator pose_estimator_ = new PoseEstimator(drivetrain_, limelight_);
  private final LED led_ = new LED();

  // Commands - (needed for autobalancing)
  private final DriveBalance drive_balance_ = new DriveBalance(drivetrain_);

  // Auto Selector
  private final AutoSelector auto_selector_ = new AutoSelector();

  // Xbox Controller
  private final XboxController driver_controller_ = new XboxController(0);
  private final XboxController operator_controller_ = new XboxController(1);

  // Telemetry
  private final Telemetry telemetry_ = new Telemetry(drivetrain_, pose_estimator_, limelight_,
      auto_selector_);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // Set default commands
    drivetrain_.setDefaultCommand(new DriveTeleop(drivetrain_, driver_controller_));
  }

  @Override
  public void robotPeriodic() {
    // Run command scheduler
    CommandScheduler.getInstance().run();

    new Trigger(driver_controller_::getAButton).onTrue(new RunCommand(() -> grabber_.setPivot(true)));
    new Trigger(driver_controller_::getBButton).onTrue(new RunCommand(() -> grabber_.setPivot(false)));

    // Run telemetry periodic functions
    telemetry_.periodic();
    updateLEDs();
    setupTeleopControls();
  
  }

  @Override
  public void autonomousInit() {
    // Set drivetrain brake mode
    drivetrain_.setBrakeMode(true);

    // Run auto
    //auto_selector_.run(drivetrain_, pose_estimator_);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Set drivetrain brake mode
    drivetrain_.setBrakeMode(true);
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
    // Set drivetrain coast mode after 5 sec
    //new DriveBrakeMode(drivetrain_).schedule();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private void setupTeleopControls(){
    //Driver Controller
    /*Arcade Drive --> Left Joystick
     * Quick Turn --> X Button
     * Pick-up from ground with grabber open --> A Button
     * Pick-up from substation with grabber open --> Y Button
     * Hold object position --> B Button
     */

    //Operator Controller
    /*Grabber Close and Open --> Left Bumper
     * Grabber Intake --> Right Bumper
     * Grabber Eject --> Right Trigger
     * Score High Level --> Up Arrow
     * Score Mid Level --> Down Arrow
     * Go into Balance Mode (with all subsystems in) --> Start Button
     *     Balance Mode means that the robot will move slower based off driver input
     */
  }

  /**
   * Updates the status of the LEDs periodically based on the various states of the robot (e.g.
   * limelight working, arm, autobalancing).
   */

  // For future reference, find a way to detect errors and add them to the requirement for LED change
  public void updateLEDs() {
    if(isDisabled()){
      //System.out.println("LEDs: Robot is disabled");
      led_.setOutput(LED.OutputType.DISABLED_READY);
    }

    else if(isEnabled()){
      //System.out.println("LEDs: Robot is enabled");
      led_.setOutput(LED.OutputType.ENABLED_READY);
    }

    else if(limelight_.hasTarget()){
      //System.out.println("LEDs: Intake");
      led_.setOutput(LED.StandardLEDOutput.LIMELIGHT_ERROR);
    }

    else if(drive_balance_.isFinished()){
      //System.out.println("LEDs: Balanced");
      led_.setOutput(LED.StandardLEDOutput.AUTOBALANCING);
    }
  }



}
