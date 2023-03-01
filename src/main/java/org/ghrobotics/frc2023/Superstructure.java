// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import org.ghrobotics.frc2023.subsystems.Arm;
import org.ghrobotics.frc2023.subsystems.Elevator;
import org.ghrobotics.frc2023.subsystems.Extender;

/** Add your docs here. */

public class Superstructure {

    private final Elevator elevator_;
    private final Extender extender_;
    private final Arm arm_;

    public Superstructure() {

        elevator_ = new Elevator();
        extender_ = new Extender();
        arm_ = new Arm();
    }

    public static enum SuperstructureState {
        SCORE_HIGH,
        SCORE_MID,
        PICKUP_GROUND,
        PICKUP_SUBSTATION,
        RESET
    }

    public SuperstructureState getWantedState() {
        // add logic
        return SuperstructureState.RESET;  // placeholder
    }

    public void periodic() {

        elevator_.setPosition(getWantedState());
        extender_.setPosition(getWantedState());
        arm_.setPosition(getWantedState());

    }
}