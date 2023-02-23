// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

/** Add your docs here. */

public class Superstructure {

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
}