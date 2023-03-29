package org.ghrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.ghrobotics.frc2023.subsystems.Elevator;

public class ElevateToPosition extends CommandBase {
  // Subsystems
  private final Elevator elevator_;

  // Position
  private final DoubleSupplier position_;

  // Constructor
  public ElevateToPosition(Elevator elevator, double position) {
    this(elevator, () -> position);
  }

  // Supplier Constructor
  public ElevateToPosition(Elevator elevator, DoubleSupplier position) {
    // Assign member variables
    elevator_ = elevator;
    position_ = position;

    // Add subsystem requirements
    addRequirements(elevator_);
  }

  @Override
  public void initialize() {
    elevator_.setPosition(position_.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return Math.abs(elevator_.getPosition() - position_.getAsDouble()) < Constants.kTolerance;
  }

  // Constants
  private static final class Constants {
    public static final double kTolerance = 0.051;
  }
}
