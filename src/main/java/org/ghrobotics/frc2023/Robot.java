// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.ghrobotics.frc2023.auto.AutoSelector;
import org.ghrobotics.frc2023.auto.ScoreBackwardThenPickup;
import org.ghrobotics.frc2023.auto.ScoreOneAndTaxi;
import org.ghrobotics.frc2023.commands.DriveBrakeMode;
import org.ghrobotics.frc2023.commands.DriveTeleop;
import org.ghrobotics.frc2023.subsystems.Arm;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.Elevator;
import org.ghrobotics.frc2023.subsystems.Extender;
import org.ghrobotics.frc2023.subsystems.Grabber;
import org.ghrobotics.frc2023.subsystems.LED;
import org.ghrobotics.frc2023.subsystems.Limelight;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Balance Mode
  private boolean balance_mode_ = false;

  // Subsystems
  private final Drivetrain drivetrain_ = new Drivetrain(() -> balance_mode_);
  private final Limelight limelight_ = new Limelight("limelight");
  private final Elevator elevator_ = new Elevator();
  private final Extender extender_ = new Extender();
  private final Arm arm_ = new Arm();
  private final Grabber grabber_ = new Grabber();
  private final PoseEstimator pose_estimator_ = new PoseEstimator(drivetrain_, limelight_);
  private final LED led_ = new LED();

  // Superstructure
  private final Superstructure superstructure_ = new Superstructure(elevator_, extender_, arm_,
      grabber_);

  // Auto Selector
  private final AutoSelector auto_selector_ = new AutoSelector();

  // Xbox Controllers
  private final CommandXboxController driver_controller_ = new CommandXboxController(0);
  private final CommandXboxController operator_controller_ = new CommandXboxController(1);

  // Telemetry
  private final Telemetry telemetry_ = new Telemetry(drivetrain_, elevator_, extender_, arm_,
      pose_estimator_, limelight_, auto_selector_);

  @Override
  public void robotInit() {
    // Set default commands
    drivetrain_.setDefaultCommand(new DriveTeleop(drivetrain_, driver_controller_));

    // Set teleop controls
    setupTeleopControls();
  }

  @Override
  public void robotPeriodic() {
    // Run command scheduler
    CommandScheduler.getInstance().run();

    // Run telemetry periodic functions
    telemetry_.periodic();
    updateLEDs();
  }

  @Override
  public void autonomousInit() {
    // Set drivetrain brake mode
    drivetrain_.setBrakeMode(true);

    // Calibrate drivetrain pitch
    drivetrain_.calibratePitch();

    // Run auto
    new ScoreBackwardThenPickup(drivetrain_, superstructure_, pose_estimator_,
      DriverStation.getAlliance()).schedule();
//    new ScoreOneAndTaxi(drivetrain_, superstructure_, pose_estimator_,
//        DriverStation.getAlliance()).schedule();
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

  private void setupTeleopControls() {
    // Driver Controller
    //  * B:  Balance Mode
    driver_controller_.b().onTrue(new InstantCommand(() -> balance_mode_ = !balance_mode_));
    //  * LB: Intake Cone
    driver_controller_.leftBumper().whileTrue(superstructure_.setGrabber(() -> 0.4, true));
    //  * LT: Outtake Cone
    driver_controller_.leftTrigger(0.2).whileTrue(superstructure_.setGrabber(() -> 0.0, true));
    //  * RB: Intake Cube
    driver_controller_.rightBumper().whileTrue(superstructure_.setGrabber(() -> 0.4, true));
    //  * RT: Outtake Cube
    driver_controller_.rightTrigger(0.2).whileTrue(
        superstructure_.setGrabber(driver_controller_::getRightTriggerAxis, false));

    // Operator Controller
    //  * Y:       L3 Cone
    operator_controller_.y().onTrue(superstructure_.setPosition(Superstructure.Position.CONE_L3));
    //  * B:       L2 Cone
    operator_controller_.b().onTrue(superstructure_.setPosition(Superstructure.Position.CONE_L2));
    //  * A:       L1 Cone / Cube / Intake
    operator_controller_.a().onTrue(superstructure_.setPosition(Superstructure.Position.INTAKE));
    //  * X:       Stow
    operator_controller_.x().onTrue(superstructure_.setPosition(Superstructure.Position.STOW));
    //  * LB:      Substation
    operator_controller_.leftBumper().onTrue(
        superstructure_.setPosition(Superstructure.Position.SUBSTATION));
    //  * POV 0:   L3 Cube
    operator_controller_.pov(0).onTrue(
        superstructure_.setPosition(Superstructure.Position.CUBE_L3));
    //  * POV 90:  L2 Cube
    operator_controller_.pov(90).onTrue(
        superstructure_.setPosition(Superstructure.Position.CUBE_L2));
    //  * POV 270: Backward Cube
    operator_controller_.pov(270).onTrue(
        superstructure_.setPosition(Superstructure.Position.BACK_EXHAUST));
  }

  /**
   * Updates the status of the LEDs periodically based on the various states of the robot (e.g.
   * limelight working, arm, autobalancing).
   */

  // For future reference, find a way to detect errors and add them to the requirement for LED
  // change
  public void updateLEDs() {
    if (isDisabled()) {
      //System.out.println("LEDs: Robot is disabled");
      led_.setOutput(LED.OutputType.DISABLED_READY);
    } else if (isEnabled()) {
      //System.out.println("LEDs: Robot is enabled");
      led_.setOutput(LED.OutputType.ENABLED_READY);
    } else if (limelight_.hasTarget()) {
      //System.out.println("LEDs: Intake");
      led_.setOutput(LED.StandardLEDOutput.LIMELIGHT_ERROR);
    } else{
      led_.setOutput(LED.StandardLEDOutput.BLANK);
    }
  }
}
