package org.ghrobotics.frc2023.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import org.ghrobotics.frc2023.Superstructure;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

public class AutoSelector {
  // Sendable Choosers
  private final SendableChooser<Side> side_chooser_;
  private final SendableChooser<Height> height_chooser_;
  private final SendableChooser<Balance> balance_chooser_;
  private final SendableChooser<Routine> routine_chooser_;

  // Constructor
  public AutoSelector() {
    // Initialize side chooser
    side_chooser_ = new SendableChooser<>();
    side_chooser_.addOption("Left", Side.LEFT);
    side_chooser_.setDefaultOption("Center", Side.CENTER);
    side_chooser_.addOption("Right", Side.RIGHT);

    // Initialize height chooser
    height_chooser_ = new SendableChooser<>();
    height_chooser_.addOption("Low", Height.LOW);
    height_chooser_.setDefaultOption("Middle", Height.MIDDLE);
    height_chooser_.addOption("High", Height.HIGH);

    // Initialize balance chooser
    balance_chooser_ = new SendableChooser<>();
    balance_chooser_.setDefaultOption("Yes", Balance.YES);
    balance_chooser_.addOption("No", Balance.NO);

    routine_chooser_ = new SendableChooser<>();
    routine_chooser_.setDefaultOption("Score One and Taxi", Routine.SCOREONETAXI);


  }

  // Side Chooser Getter
  public SendableChooser<Side> getSideChooser() {
    return side_chooser_;
  }

  // Height Chooser Getter
  public SendableChooser<Height> getHeightChooser() {
    return height_chooser_;
  }

  // Balance Chooser Getter
  public SendableChooser<Balance> getBalanceChooser() {
    return balance_chooser_;
  }

  // Run Auto
  public void run(Drivetrain drivetrain, PoseEstimator pose_estimator) {
    /*new ScoreOne(drivetrain, pose_estimator, side_chooser_.getSelected(),
        height_chooser_.getSelected(), balance_chooser_.getSelected()).schedule();*/
  }

  public enum Side {
    LEFT, CENTER, RIGHT
  }

  public enum Height {
    LOW, MIDDLE, HIGH
  }

  public enum Balance {
    YES, NO
  }

  public enum Routine {
    SCOREONETAXI
  }
}
