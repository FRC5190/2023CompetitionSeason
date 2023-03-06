package org.ghrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2023.subsystems.Extender;

public class ExtendToPosition extends CommandBase {
  // Subsystems
  private final Extender extender_;

  // Position
  private final double position_;

  // Constructor
  public ExtendToPosition(Extender extender, double position) {
    // Assign member variables
    extender_ = extender;
    position_ = position;

    // Add subsystem requirements
    addRequirements(extender_);
  }

  @Override
  public void initialize() {
    extender_.setPosition(position_);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(extender_.getPosition() - position_) < Constants.kTolerance;
  }

  // Constants
  private static final class Constants {
    public static final double kTolerance = 0.0255;
  }
}
