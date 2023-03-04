// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import org.ghrobotics.frc2023.subsystems.Grabber;

/**
 * Add your docs here.
 */
public class Superstructure {

    //Subsystems
    private final Grabber grabber_;

    //Subsystem States
    private GrabberState grabber_state_ = GrabberState.IDLE;

    //Subsystem References
    private double grabber_pct_;
    private boolean grabber_pivot_;

    public Superstructure(Grabber grabber){
        grabber_ = grabber;

    }


    public void update(){
        switch (grabber_state_){
            case IDLE:
                grabber_pivot_ = true;
                grabber_pct_ = Constants.kIdleGrabberPct;
                break;
            case INTAKE:
                grabber_pivot_ = true;
                //grabber_pct_ = Constants.kIntakeGrabberPct;
                break;
            case EJECT:
                grabber_pivot_ = true;
                grabber_pct_ = Constants.kEjectGrabberPct;
                break;
            case RELEASE:
                grabber_pivot_ = false;
                grabber_pct_ = Constants.kIdleGrabberPct;
                break;
            case HOLD:
                grabber_pivot_ = true;
                grabber_pct_ = Constants.kIdleGrabberPct;
        }

        //grabber_.setPercent(grabber_pct_);
        //grabber_.setPivot(grabber_pivot_);

    }

    public void scoreHigh() {

    }

    public void scoreMid() {

    }

    public void setHint(){

    }
    
    public enum GrabberState{
        IDLE, INTAKE, EJECT, RELEASE, HOLD
    }

    public static class Constants {
         //Grabber
        public static final int kIdleGrabberPct = 0;
        public static final double kIntakeGrabberPct = 0.2;
        public static final double kEjectGrabberPct = 0.4;
    }
}
