package org.ghrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2023.subsystems.Grabber;

public class GrabberPercent extends CommandBase {
  // Subsystems
  private final Grabber grabber_;

  // Percent
  private final double percent_;

  // Constructor
  public GrabberPercent(Grabber grabber, double percent) {
    // Assign member variables
    grabber_ = grabber;
    percent_ = percent;
  }

  @Override
  public void initialize() {
    grabber_.setPercent(percent_);
  }

  @Override
  public void end(boolean interrupted) {
    grabber_.setPercent(0);
  }
}
