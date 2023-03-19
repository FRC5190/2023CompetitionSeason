package org.ghrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2023.subsystems.Elevator;

public class ElevateToPosition extends CommandBase {
  // Subsystems
  private final Elevator elevator_;

  // Position
  private final double position_;

  // Constructor
  public ElevateToPosition(Elevator elevator, double position) {
    // Assign member variables
    elevator_ = elevator;
    position_ = position;

    // Add subsystem requirements
    addRequirements(elevator_);
  }

  @Override
  public void execute() {
    elevator_.setPosition(position_);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(elevator_.getPosition() - position_) < Constants.kTolerance;
  }

  // Constants
  private static final class Constants {
    public static final double kTolerance = 0.051;
  }
}
