// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ghrobotics.frc2023.auto.AutoSelector;
import org.ghrobotics.frc2023.commands.DriveBrakeMode;
import org.ghrobotics.frc2023.commands.DriveTeleop;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
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
  private final PoseEstimator pose_estimator_ = new PoseEstimator(drivetrain_, limelight_);
  private final LED led_ = new LED();

  // Commands - (needed for autobalancing)
  private final DriveBalance drive_balance_ = new DriveBalance(drivetrain_);

  // Auto Selector
  private final AutoSelector auto_selector_ = new AutoSelector();

  // Xbox Controller
  private final XboxController driver_controller_ = new XboxController(0);

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

    // Run telemetry periodic functions
    telemetry_.periodic();
  }

  @Override
  public void autonomousInit() {
    // Set drivetrain brake mode
    drivetrain_.setBrakeMode(true);

    // Run auto
    auto_selector_.run(drivetrain_, pose_estimator_);
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
    new DriveBrakeMode(drivetrain_).schedule();
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

  /**
   * Updates the status of the LEDs periodically based on the various states of the robot (e.g.
   * limelight working, arm, autobalancing).
   */

  // For future reference, find a way to detect errors and add them to the requirement for LED change
  public void updateLEDs() {
    if(isDisabled()){
      System.out.println("LEDs: Robot is disabled");
      led_.setOutput(LED.OutputType.DISABLED_READY);
    }

    else if(isEnabled()){
      System.out.println("LEDs: Robot is enabled");
      led_.setOutput(LED.OutputType.ENABLED_READY);
    }

    else if(limelight_.hasTarget()){
      System.out.println("LEDs: Intake");
      led_.setOutput(LED.StandardLEDOutput.LIMELIGHT_ERROR);
    }

    else if(drive_balance_.isFinished()){
      System.out.println("LEDs: Balanced");
      led_.setOutput(LED.StandardLEDOutput.AUTOBALANCING);
    }
  }
  


}
