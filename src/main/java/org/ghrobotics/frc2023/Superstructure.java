// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import org.ghrobotics.frc2023.subsystems.Arm;
import org.ghrobotics.frc2023.subsystems.Elevator;
import org.ghrobotics.frc2023.subsystems.Extender;
import org.ghrobotics.frc2023.subsystems.Grabber;

/**
 * Add your docs here.
 */
public class Superstructure {

    //Subsystems
    private final Elevator elevator_;
    private final Extender extender_;
    private final Arm arm_;
    private final Grabber grabber_;

    //Subsystem States
    private ElevatorState elevator_state_ = ElevatorState.IDLE;
    private ExtenderState extender_state_ = ExtenderState.IDLE;
    private ArmState arm_state_ = ArmState.IDLE;
    private GrabberState grabber_state_ = GrabberState.IDLE;

    //Subsystem References
    private double grabber_pct_;
    private boolean grabber_pivot_;

    public Superstructure(Elevator elevator, Extender extender, Arm arm, Grabber grabber){
        elevator_ = elevator;
        extender_ = extender;
        arm_ = arm;
        grabber_ = grabber;

    }


    public void update(){
        switch (grabber_state_){
            case IDLE:
                grabber_pivot_ = false;
                grabber_pct_ = Constants.kIdleGrabberPct;
                break;
            case INTAKE:
                grabber_pivot_ = true;
                grabber_pct_ = Constants.kIntakeGrabberPct;
                break;
            case EJECT:
                grabber_pivot_ = true;
                grabber_pct_ = Constants.kEjectGrabberPct;
        }

        grabber_.setPercent(grabber_pct_);
        grabber_.setPivot(grabber_pivot_);

    }

    public void scoreHigh() {
        elevator_.setPosition(Constants.kScoreHighElevatorPos);
        extender_.setPosition(Constants.kScoreHighExtenderPos);
        // arm_.setPosition(Constants.kScoreHighArmPos);

    }

    public void scoreMid() {
        elevator_.setPosition(Constants.kScoreMidElevatorPos);
        extender_.setPosition(Constants.kScoreMidExtenderPos);
        // arm_.setPosition(Constants.kScoreMidArmPos);
    }

    public void scoreLow() {
        elevator_.setPosition(Constants.kScoreLowElevatorPos);
        extender_.setPosition(Constants.kScoreLowExtenderPos);
        // arm_.setPosition(Constants.kScoreLowArmPos);
    }

    public void pickupGround() {
        elevator_.setPosition(Constants.kPickupGroundElevatorPos);
        extender_.setPosition(Constants.kPickupGroundExtenderPos);
        // arm_.setPosition(Constants.kPickupGroundArmPos);
    }

    public void pickupSubstation() {
        elevator_.setPosition(Constants.kPickupSubstationElevatorPos);
        extender_.setPosition(Constants.kPickupSubstationExtenderPos);
        // arm_.setPosition(Constants.kPickupSubstationArmPos);
    }

    public void reset() {
        elevator_.setPosition(Constants.kIdleElevatorPos);
        extender_.setPosition(Constants.kIdleExtenderPos);
        // arm_.setPosition(Constants.kIdleArmPos);
    }

    public void setHint(){
        // whaT
    }

    public enum ElevatorState {
        IDLE, SCORE_HIGH, SCORE_MID, SCORE_LOW, PICKUP_GROUND, PICKUP_SUBSTATION
    }

    public enum ExtenderState {
        IDLE, SCORE_HIGH, SCORE_MID, SCORE_LOW, PICKUP_GROUND, PICKUP_SUBSTATION
    }

    public enum ArmState {
        IDLE, SCORE_HIGH, SCORE_MID, SCORE_LOW, PICKUP_GROUND, PICKUP_SUBSTATION
    }
    
    public enum GrabberState{
        IDLE, INTAKE, EJECT
    }

    public static class Constants {
        
        // Elevator
        public static final double kIdleElevatorPos = 0;
        public static final double kScoreHighElevatorPos = 0;
        public static final double kScoreMidElevatorPos = 0;
        public static final double kScoreLowElevatorPos = 0;
        public static final double kPickupGroundElevatorPos = 0;
        public static final double kPickupSubstationElevatorPos = 0;

        // Extender
        public static final double kIdleExtenderPos = 0;
        public static final double kScoreHighExtenderPos = 0;
        public static final double kScoreMidExtenderPos = 0;
        public static final double kScoreLowExtenderPos = 0;
        public static final double kPickupGroundExtenderPos = 0;
        public static final double kPickupSubstationExtenderPos = 0;

        // Arm
        public static final double kIdleArmPos = 0;
        public static final double kScoreHighArmPos = 0;
        public static final double kScoreMidArmPos = 0;
        public static final double kScoreLowArmPos = 0;
        public static final double kPickupGroundArmPos = 0;
        public static final double kPickupSubstationArmPos = 0;

        // Grabber
        public static final int kIdleGrabberPct = 0;
        public static final double kIntakeGrabberPct = 0.2;
        public static final double kEjectGrabberPct = 0.4;

    }
}
