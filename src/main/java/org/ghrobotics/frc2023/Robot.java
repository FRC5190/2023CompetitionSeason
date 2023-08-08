// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.ghrobotics.frc2023.auto.AutoSelector;
import org.ghrobotics.frc2023.commands.DriveBrakeMode;
import org.ghrobotics.frc2023.commands.DriveTeleop;
import org.ghrobotics.frc2023.commands.DriveTowardPosition;
import org.ghrobotics.frc2023.commands.HomeSuperstructure;
import org.ghrobotics.frc2023.subsystems.Arm;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.Elevator;
import org.ghrobotics.frc2023.subsystems.Extender;
import org.ghrobotics.frc2023.subsystems.Grabber;
import org.ghrobotics.frc2023.subsystems.LED;
import org.ghrobotics.frc2023.subsystems.LED.StandardLEDOutput;
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
  // private final CommandXboxController operator_controller_ = new CommandXboxController(1);

  // Toggle Operator mode
  Trigger operator_mode = new Trigger(() -> false);

  // Operator Cube Modifier
  Trigger cube_modifier = driver_controller_.rightTrigger(0.4);

  // Drive to Position
  Command drive_pos_ = new DriveTowardPosition(drivetrain_, pose_estimator_,
      driver_controller_, Arena.getTagPosition(4).toPose2d());

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
    //superstructure_.periodic();
    //System.out.println(superstructure_.position_);
    updateLEDs();
  }

  @Override
  public void autonomousInit() {
    // Set drivetrain brake mode
    drivetrain_.setBrakeMode(true);

    // Run auto
    DriverStation.Alliance alliance = DriverStation.getAlliance();
    auto_selector_.run(drivetrain_, pose_estimator_, superstructure_, alliance).schedule();
    System.out.println("AUTO ENABLE");

    // Calibrate drivetrain pitch
//    drivetrain_.calibratePitch();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Set drivetrain brake mode
    drivetrain_.setBrakeMode(true);
    // pose_estimator_.resetPosition(Arena.getTagPosition(4).toPose2d());
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
    // Operator Mode Revert
    if(driver_controller_.leftTrigger(04).getAsBoolean() && driver_controller_.rightTrigger(0.4).getAsBoolean())
      operator_mode = (operator_mode.getAsBoolean()) ? new Trigger(() -> false) : new Trigger(() -> true);

    if (!operator_mode.getAsBoolean()) {
      // Driver Controller
      driver_controller_.a().whileTrue(drive_pos_);
      //  * B:  Balance Mode
      driver_controller_.b().onTrue(new InstantCommand(() -> balance_mode_ = !balance_mode_));
      //  * LB: Intake Cone
      driver_controller_.leftBumper().whileTrue(superstructure_.setGrabber(() -> -0.25, false));
      //  * LT: Outtake Cone
      driver_controller_.leftTrigger(0.2).whileTrue(
          superstructure_.setGrabber(() -> 0, true));
      //  * RB: Intake Cube
      driver_controller_.rightBumper().whileTrue(superstructure_.setGrabber(() -> -0.25, true));
      //  * RT: Outtake Cube and Cone L3
      driver_controller_.rightTrigger(0.15).whileTrue(
          superstructure_.setGrabber(() -> driver_controller_.getRightTriggerAxis() * 0.15, false));
    }

    if (operator_mode.getAsBoolean()) {
      // Operator Controller
      //  * Y:       L3 Cone / Cube
      cube_modifier.onTrue(new InstantCommand(() -> led_.setOutput(LED.StandardLEDOutput.CUBE)));
      driver_controller_.y().and(cube_modifier).onTrue(superstructure_.setPosition(
          Superstructure.Position.CUBE_L3));
      driver_controller_.y().and(cube_modifier.negate()).onTrue(superstructure_.setPosition(
          Superstructure.Position.CONE_L2));
      //  * B:       L2 Cone / Cube
      driver_controller_.b().and(cube_modifier).onTrue(superstructure_.setPosition(
          Superstructure.Position.CUBE_L2));
      driver_controller_.b().and(cube_modifier.negate()).onTrue(superstructure_.setPosition(
          Superstructure.Position.CONE_L2));
      //  * A:       L1 Cone / Cube / Intake
      driver_controller_.a().onTrue(superstructure_.setPosition(Superstructure.Position.INTAKE));
      //  * X:       Stow
      driver_controller_.x().onTrue(superstructure_.setPosition(Superstructure.Position.STOW));
      //  * LB:      Substation
      driver_controller_.leftBumper().onTrue(
          superstructure_.setPosition(Superstructure.Position.SUBSTATION));
      //  * Back:    Superstructure Reset
      driver_controller_.back().onTrue(new HomeSuperstructure(elevator_, extender_, arm_));
      //  * POV 0: Manual Elevator Up
      driver_controller_.pov(0).whileTrue(superstructure_.jogElevator(0.25));
      // * POV 180: Manual Elevator Down
      driver_controller_.pov(180).whileTrue(superstructure_.jogElevator(-0.25));
    }

  }

  /**
   * Updates the status of the LEDs periodically based on the various states of the robot (e.g.
   * limelight working, arm, autobalancing).
   */

  // For future reference, find a way to detect errors and add them to the requirement for LED
  // change
  public void updateLEDs() {
    // Disabled State
    if (isDisabled()) {
      // Limelight Not Connected
      led_.setOutput(LED.OutputType.DISABLED_READY);
    } else if (isAutonomous()) {
      led_.setOutput(StandardLEDOutput.AUTO);
    } else {
      // Cube Modifier
      if (cube_modifier.getAsBoolean()) {
        led_.setOutput(
            drive_pos_.isScheduled() ? StandardLEDOutput.CUBE_ALIGNING : StandardLEDOutput.CUBE);
      } else {
        // Default
        led_.setOutput(
            drive_pos_.isScheduled() ? StandardLEDOutput.CONE_ALIGNING : StandardLEDOutput.CONE);
      }
    }
  }
}
