package org.ghrobotics.frc2023.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import org.ghrobotics.frc2023.Superstructure;
import org.ghrobotics.frc2023.subsystems.Drivetrain;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;

public class AutoSelector {
  private Command autonomous_command_ = null;

  // Sendable Choosers
  private final SendableChooser<Routine> routine_chooser_;
  private final SendableChooser<Grid> grid_chooser_;
 // private final SendableChooser<Side> side_chooser_;
 // private final SendableChooser<Height> height_chooser_;
 // private final SendableChooser<Balance> balance_chooser_;

  // Constructor
  public AutoSelector() {
    //Intialize routine chooser
    routine_chooser_ = new SendableChooser<>();
    routine_chooser_.setDefaultOption("Score Coop Balance", Routine.SCORECOOPBALANCE);
    routine_chooser_.addOption("Score Cone Then Cube", Routine.SCORE_CONE_THEN_CUBE);
    routine_chooser_.addOption("Score Forward Exit Balance Backwards", Routine.SCOREFORWARDEXITBALANCEBACKWARD);
    //routine_chooser_.addOption("Score One and Taxi", Routine.SCOREONETAXI);
    //routine_chooser_.addOption("Score Forward and Balance Backward", Routine.SCOREFORWARDBALANCEBACKWARD);
    //routine_chooser_.addOption("Score Two and Taxi", Routine.SCORETWOTAXI);
    //routine_chooser_.addOption("Score Backward then Pickup and Balance", Routine.SCOREBACKWARDPICKUPBALANCE);
    //routine_chooser_.addOption("Score Backward and Balance", Routine.SCOREBACKWARDBALANCE);

    //Initialize grid chooser
    grid_chooser_ = new SendableChooser<>();
    grid_chooser_.setDefaultOption("Top", Grid.TOP);
    grid_chooser_.addOption("Bottom", Grid.BOTTOM);

/*
    // Initialize side chooser
    side_chooser_ = new SendableChooser<>();
    side_chooser_.setDefaultOption(" ", null);
    side_chooser_.addOption("Left", Side.LEFT);
    side_chooser_.addOption("Center", Side.CENTER);
    side_chooser_.addOption("Right", Side.RIGHT);

    // Initialize height chooser
    height_chooser_ = new SendableChooser<>();
    height_chooser_.setDefaultOption(" ", null);
    height_chooser_.addOption("Low", Height.LOW);
    height_chooser_.addOption("Middle", Height.MIDDLE);
    height_chooser_.addOption("High", Height.HIGH);

    // Initialize balance chooser
    balance_chooser_ = new SendableChooser<>();
    balance_chooser_.setDefaultOption(" ", null);
    balance_chooser_.addOption("Yes", Balance.YES);
    balance_chooser_.addOption("No", Balance.NO);
    */

  }

  //Routine Chooser Getter
  public SendableChooser<Routine> getRoutineChooser() {
    return routine_chooser_;
  }

  //Grid Chooser Getter
  public SendableChooser<Grid> getGridChooser() {
    return grid_chooser_;
  }
/*
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
  }*/

  // Run Auto
  public Command run(Drivetrain drivetrain, PoseEstimator pose_estimator, Superstructure superstructure, DriverStation.Alliance alliance) {
    if (routine_chooser_.getSelected() == Routine.SCOREFORWARDEXITBALANCEBACKWARD) {
      autonomous_command_ = new ScoreForwardExitBalanceBackwards(drivetrain, superstructure, pose_estimator, alliance, grid_chooser_.getSelected());
      return autonomous_command_;
    }
    else if (routine_chooser_.getSelected() == Routine.SCORECOOPBALANCE) {
      autonomous_command_ = new ScoreCoopertitionBalanceBackwards(drivetrain, superstructure, pose_estimator, alliance);
      return autonomous_command_;
    }
    else if (routine_chooser_.getSelected() == Routine.SCORE_CONE_THEN_CUBE) {
      autonomous_command_ = new ScoreConeThenCube(drivetrain, superstructure, pose_estimator, alliance);
      return autonomous_command_;
    }
    else {
      return new ScoreConeThenCube(drivetrain, superstructure, pose_estimator, alliance);
    }
    /*new ScoreOne(drivetrain, pose_estimator, side_chooser_.getSelected(),
        height_chooser_.getSelected(), balance_chooser_.getSelected()).schedule();*/
  }

  public enum Routine {
    SCOREFORWARDEXITBALANCEBACKWARD,
    SCORECOOPBALANCE,
    SCORE_CONE_THEN_CUBE
  }

  public enum Grid {
    TOP, BOTTOM //scoring table near bottom
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

}
