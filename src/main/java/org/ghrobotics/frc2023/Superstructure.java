package org.ghrobotics.frc2023;

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
