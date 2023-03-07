package org.ghrobotics.frc2023;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.ghrobotics.frc2023.commands.ArmToPosition;
import org.ghrobotics.frc2023.commands.ElevateToPosition;
import org.ghrobotics.frc2023.commands.ExtendToPosition;
import org.ghrobotics.frc2023.commands.GrabberPercent;
import org.ghrobotics.frc2023.subsystems.Arm;
import org.ghrobotics.frc2023.subsystems.Elevator;
import org.ghrobotics.frc2023.subsystems.Extender;
import org.ghrobotics.frc2023.subsystems.Grabber;

public class Superstructure {
  // Subsystems
  private final Elevator elevator_;
  private final Extender extender_;
  private final Arm arm_;
  private final Grabber grabber_;

  // Constructor
  public Superstructure(Elevator elevator, Extender extender, Arm arm, Grabber grabber) {
    // Assign member variables
    elevator_ = elevator;
    extender_ = extender;
    arm_ = arm;
    grabber_ = grabber;
  }

  // Position Setter
  public Command setPosition(Position pos) {
    // Find the arm's "elevator movement" position -- where the arm should be when the
    // elevator is moving. This is max(kElevatorMovementPosition, desired arm position).
    double arm_elev_mvmt_pos = Math.max(pos.angle, Constants.kElevatorMovementArmPosition);

    // Create and return command group
    return new SequentialCommandGroup(
        // Take elevator to desired height while keeping the arm at the "elevator movement" pos.
        // Also, bring extension back in. End this when we reach the desired elevator height.
        new ParallelDeadlineGroup(new ElevateToPosition(elevator_, pos.height),
            new ExtendToPosition(extender_, Constants.kExtenderStowPosition),
            new ArmToPosition(arm_, arm_elev_mvmt_pos)
        ),

        // Take extender and arm to final position.
        new ParallelCommandGroup(
            new ExtendToPosition(extender_, pos.extension),
            new ArmToPosition(arm_, pos.angle)
        ));
  }

  public Command setGrabber(double percent, boolean pivot) {
    return new GrabberPercent(grabber_, percent, pivot);
  }

  // Positions
  public enum Position {
    // Stowed position, everything inside the robot
    STOW(0, 0, 125),

    // Intaking a game piece
    INTAKE(0, 0, -20),

    // Exhaust cube out the back of the robot
    BACK_EXHAUST(31, 0, 125),

    // Testing
    TEST(20, 5, 30),

    //Score High
    SCOREHIGH(20, 5, 30),

    //Substation
    SUBSTATION(31, 0, 10);

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
