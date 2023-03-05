package org.ghrobotics.frc2023;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.ghrobotics.frc2023.subsystems.Arm;
import org.ghrobotics.frc2023.subsystems.Elevator;
import org.ghrobotics.frc2023.subsystems.Extender;

public class Superstructure {
  // Subsystems
  private final Elevator elevator_;
  private final Extender extender_;
  private final Arm arm_;

  // Constructor
  public Superstructure(Elevator elevator, Extender extender, Arm arm) {
    // Assign member variables
    elevator_ = elevator;
    extender_ = extender;
    arm_ = arm;
  }

  // Position Setter
  public Command setPosition(Position pos) {
    // Find the arm's "elevator movement" position -- where the arm should be when the
    // elevator is moving. This is max(kElevatorMovementPosition, desired arm position)
    double arm_elev_mvmt_pos = Math.max(pos.angle, Constants.kElevatorMovementArmPosition);

    // Create and return command group
    return new SequentialCommandGroup(
        new PrintCommand("in here"),
        // Take elevator to desired height while keeping the arm at the "elevator movement" pos.
        // Also, bring extension back in
        new InstantCommand(() -> elevator_.setPosition(pos.height)),
        new InstantCommand(() -> arm_.setAngle(arm_elev_mvmt_pos)),
        new InstantCommand(() -> extender_.setPosition(Constants.kExtenderStowPosition)),

        // Wait for the elevator to reach desired position within tolerance
        new WaitUntilCommand(() -> Math.abs(
            pos.height - elevator_.getPosition()) < Constants.kElevatorHeightTolerance),

        // Take extender and arm to final position
        new InstantCommand(() -> extender_.setPosition(pos.extension)),
        new InstantCommand(() -> arm_.setAngle(pos.angle))
    );
  }

  // Positions
  public enum Position {
    INTAKE(0, 0, -20),
    TEST(31, 9, 30);

    final double height;
    final double extension;
    final double angle;

    Position(double height_in, double extension_in, double angle_deg) {
      this.height = Units.inchesToMeters(height_in);
      this.extension = Units.inchesToMeters(extension_in);
      this.angle = Math.toRadians(angle_deg);
    }
  }

  // Constants
  private static class Constants {
    // Tolerances
    public static final double kElevatorHeightTolerance = 0.051;

    // Misc
    public static final double kExtenderStowPosition = 0.0;
    public static final double kElevatorMovementArmPosition = Math.PI / 2;
  }
}
