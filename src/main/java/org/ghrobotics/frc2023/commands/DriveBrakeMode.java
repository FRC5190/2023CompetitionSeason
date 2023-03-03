package org.ghrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.ghrobotics.frc2023.subsystems.Drivetrain;

public class DriveBrakeMode extends SequentialCommandGroup {
  // Constructor
  public DriveBrakeMode(Drivetrain drivetrain) {
    // Add sequence of commands -> wait 5 seconds, set brake mode to false.
    addCommands(
        new WaitCommand(5),
        new InstantCommand(() -> drivetrain.setBrakeMode(false))
    );
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
