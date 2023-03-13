package org.ghrobotics.frc2023;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.time.Instant;
import java.util.function.DoubleSupplier;
import org.ghrobotics.frc2023.commands.ArmToPosition;
import org.ghrobotics.frc2023.commands.ElevateToPosition;
import org.ghrobotics.frc2023.commands.ExtendToPosition;
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
  
  //Store Position
  public String state = "STOW";

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
    this.state = pos.posname;
    //SmartDashboard.putString("Superstructure Position",this.state);

    // Find the arm's "elevator movement" position -- where the arm should be when the
    // elevator is moving. This is max(kElevatorMovementPosition, desired arm position).
    double arm_elev_mvmt_pos = Math.max(pos.angle, Constants.kElevatorMovementArmPosition);

    // Create and return command group
    return new SequentialCommandGroup(
        new InstantCommand(() -> this.state = pos.posname),
        // Take elevator to desired height while keeping the arm at the "elevator movement" pos.
        // Also, bring extension back in. End this when we reach the desired elevator height.
        new ParallelDeadlineGroup(new ElevateToPosition(elevator_, pos.height),
          //new PrintCommand("Inside Parallel Deadine Group"),
            new ExtendToPosition(extender_, Constants.kExtenderStowPosition),
            new ArmToPosition(arm_, arm_elev_mvmt_pos)
        ),

        // Take extender and arm to final position.
        new ParallelCommandGroup(
          //new PrintCommand("Inside Parallel Command Group"),
            new ExtendToPosition(extender_, pos.extension),
            new ArmToPosition(arm_, pos.angle))
        );
  }

  //GetPosition of Superstructure
  public String getState()
  {
    return state;
  }

  // Grabber Setter
  public Command setGrabber(DoubleSupplier percent, boolean open) {
    return new FunctionalCommand(
        // Initialize
        () -> grabber_.setPivot(open),

        // Execute
        () -> grabber_.setPercent(percent.getAsDouble()),

        // End
        (interrupted) -> {
          grabber_.setPercent(0);
          grabber_.setPivot(false);
        },

        // Is Finished
        () -> false,

        // Subsystem Requirement
        grabber_
    );
  }

  // Positions
  public enum Position {
    // Stowed position, everything inside the robot
    STOW(0, 0, 125,"STOW"),

    // Intaking a game piece
    INTAKE(0, 0, -20,"INTAKE"),

    // Exhaust cube out the back of the robot
    BACK_EXHAUST(29, 0, 120,"BACK_EXHAUST"),

    // Pick up from substation
    SUBSTATION(32, 0, 5,"SUBSTATION"),

    // Cube scoring
    CUBE_L2(20, 6, 10,"CUBE_L2"),
    CUBE_L3(27, 6, 20,"CUBE_L3"),

    // Cone scoring
    CONE_L2(26, 9, 5,"CONE_L2"),
    CONE_L3(30, 9, 45,"CONE_L3");

    final double height;
    final double extension;
    final double angle;
    final String posname;

    Position(double height_in, double extension_in, double angle_deg, String name) {
      this.height = Units.inchesToMeters(height_in);
      this.extension = Units.inchesToMeters(extension_in);
      this.angle = Math.toRadians(angle_deg);
      this.posname = name;
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
