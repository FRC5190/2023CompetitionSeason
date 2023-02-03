// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;


/** Add your docs here. */
public class Drivetrain extends SubsystemBase{
    //Motor Controllers
    private final CANSparkMax left_leader_;
    private final CANSparkMax left_follower_;
    private final CANSparkMax right_leader_;
    private final CANSparkMax right_follower_;

    //Output Limit
    private boolean limit_output = false;

    //IO
    private OutputType output_type_ = OutputType.PERCENT;
    private final PeriodicIO io_ = new PeriodicIO();

    public Drivetrain(){
        super();
        //Initialize motor controllers
        left_leader_ = new CANSparkMax(Constants.kLeftLeaderId, MotorType.kBrushless);
        left_leader_.setInverted(false);

        left_follower_ = new CANSparkMax(Constants.kLeftFollowerId, MotorType.kBrushless);
        left_follower_.follow(left_leader_);

        right_leader_ = new CANSparkMax(Constants.kRightLeaderId, MotorType.kBrushless);
        right_leader_.setInverted(true);

        right_follower_ = new CANSparkMax(Constants.kRightFollowerId, MotorType.kBrushless);
        right_follower_.follow(right_follower_);

    }

    @Override
    public void periodic(){
        switch (output_type_){
            case PERCENT:
                left_leader_.set(limit_output ? 
                    MathUtil.clamp(io_.l_demand, -Constants.kOutputLimit, Constants.kOutputLimit) : 
                    io_.l_demand);
                right_leader_.set(limit_output ? 
                    MathUtil.clamp(io_.r_demand, -Constants.kOutputLimit, Constants.kOutputLimit) : 
                    io_.r_demand);
        }
        //System.out.println("in drivetrain periodic");
    }

    public void setPercent(double l, double r) {
        output_type_ = OutputType.PERCENT;
        io_.l_demand = l;
        io_.r_demand = r;
    }

    enum OutputType {
        PERCENT
      }

    public static class PeriodicIO {
        //Inputs

        //Outputs
        double l_demand;
        double r_demand;
    }

    public static class Constants {
        //Motor Controller IDs
        public static final int kLeftLeaderId = 2; 
        public static final int kLeftFollowerId = 1;
        public static final int kRightLeaderId = 4;
        public static final int kRightFollowerId = 3;

        //Output Limit
        public static final double kOutputLimit = 0.3;
    }

    public void moveForwardTest(double startTime) {
        double time = Timer.getFPGATimestamp();
        System.out.println(time - startTime);

        if (time - startTime < 1) {
            left_leader_.set(0.15);
            left_follower_.set(0.15);
            right_leader_.set(-0.15);
            right_follower_.set(-0.15);
        } else {
            left_leader_.set(0);
            left_follower_.set(0);
            right_leader_.set(0);
            right_follower_.set(0);
        }
    }
}
