// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import org.ghrobotics.frc2023.subsystems.Arm;
import org.ghrobotics.frc2023.subsystems.Elevator;
import org.ghrobotics.frc2023.subsystems.Extender;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */

public class Superstructure {

    private final Elevator elevator_;
    private final Extender extender_;
    private final Arm arm_;

    private final XboxController controller_;

    private SuperstructureState current_state_;

    public Superstructure(Elevator elevator, Extender extender, Arm arm,
                XboxController controller, SuperstructureState currentState) {

        elevator_ = elevator;
        extender_ = extender;
        arm_ = arm;

        controller_ = controller;

        current_state_ = currentState;
    }

    public static enum SuperstructureState {
        SCORE_HIGH,
        SCORE_MID,
        PICKUP_GROUND,
        PICKUP_SUBSTATION,
        RESET
    }

    public SuperstructureState getWantedState() {
        SuperstructureState wantedState = current_state_;

        if (controller_.getYButton()) {
            wantedState = SuperstructureState.SCORE_HIGH;
        }
        else if (controller_.getAButton()) {
            wantedState = SuperstructureState.SCORE_MID;
        }
        else if (controller_.getXButton()) {
            wantedState = SuperstructureState.PICKUP_SUBSTATION;
        }
        else if (controller_.getBButton()) {
            wantedState = SuperstructureState.PICKUP_GROUND;
        }
        else if (controller_.getRightBumper()) {
            wantedState = SuperstructureState.RESET;
        }

        current_state_ = wantedState;

        return wantedState;
    }

    public void reset() {
        elevator_.setPosition(SuperstructureState.RESET);
        extender_.setPosition(SuperstructureState.RESET);
        arm_.setPosition(SuperstructureState.RESET);
    }

    public void periodic() {
        SuperstructureState state = getWantedState();

        elevator_.setPosition(state);
        extender_.setPosition(state);
        arm_.setPosition(state);

    }
}