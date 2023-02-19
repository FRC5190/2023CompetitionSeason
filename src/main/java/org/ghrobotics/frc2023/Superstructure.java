package org.ghrobotics.frc2023;

public class Superstructure {

    public static enum RobotState {
        SCORE_HIGH,
        SCORE_MID,
        PICKUP_GROUND,
        PICKUP_SUBSTATION,
        RESET
    }

    public RobotState getCurrentState() {
        // add logic
        return RobotState.RESET;  // placeholder
    }

    
}
