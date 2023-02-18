// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ghrobotics.frc2023.subsystems.PoseEstimator;


/** Add your docs here. */
public class Drivetrain extends SubsystemBase{
    //Motor Controllers
    private final CANSparkMax left_leader_;
    private final CANSparkMax left_follower_;
    private final CANSparkMax right_leader_;
    private final CANSparkMax right_follower_;

    //Sensors
    private final RelativeEncoder left_encoder_;
    private final RelativeEncoder right_encoder_;

    //Trajectory Tracking
    public final DifferentialDriveKinematics kinematics_;

    //Output Limit
    private boolean limit_output = false;

    //IO
    private OutputType output_type_ = OutputType.PERCENT;
    private final PeriodicIO io_ = new PeriodicIO();

    private final PoseEstimator estimator_;

    public Drivetrain(){
        super();
        //Initialize motor controllers
        left_leader_ = new CANSparkMax(Constants.kLeftLeaderId, MotorType.kBrushless);
        left_leader_.setInverted(true);

        //left_follower_ = new CANSparkMax(Constants.kLeftFollowerId, MotorType.kBrushless);
        //left_follower_.follow(left_leader_);

        right_leader_ = new CANSparkMax(Constants.kRightLeaderId, MotorType.kBrushless);
        right_leader_.setInverted(false);

        //right_follower_ = new CANSparkMax(Constants.kRightFollowerId, MotorType.kBrushless);
        //right_follower_.follow(right_follower_);

        //Initialize encoders
        left_encoder_ = left_leader_.getEncoder();
        left_encoder_.setPositionConversionFactor(
            2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio);

        right_encoder_= right_leader_.getEncoder();
        right_encoder_.setPositionConversionFactor(
            2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio);

        kinematics_ = new DifferentialDriveKinematics(Constants.kTrackWidth);
    }

    @Override
    public void periodic(){
        //Read inputs
        io_.l_position = left_encoder_.getPosition();
        io_.r_position = right_encoder_.getPosition();


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

    public void setBrakeMode(boolean value){
        IdleMode mode = value ? IdleMode.kBrake : IdleMode.kCoast;
        left_leader_.setIdleMode(mode);
        left_follower_.setIdleMode(mode);
        right_leader_.setIdleMode(mode);
        right_follower_.setIdleMode(mode);
    }

    public double getLeftPosition(){
        return io_.l_position;
    }

    public double getRightPosition(){
        return io_.r_position;
    }
    

    enum OutputType {
        PERCENT
      }

    public static class PeriodicIO {
        //Inputs
        double l_position;
        double r_position;
        double l_velocity;
        double r_velocity;

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

        //Hardware
        public static double kGearRatio = 10.18;
        public static double kWheelRadius = 0.0762;
        public static double kTrackWidth = 0.759; //CHANGE
        //public static double kMass = 65.0; //CHANGE
        //public static double kMOI = 10.0; //CHANGE

        //Control
        public static double kLeftKs;
        public static double kLeftKv;
        public static double kLeftKa;
        public static double kLeftKp;
        public static double kRightKs;
        public static double kRightKv;
        public static double kRightKa;
        public static double kRightKp;

        //Output Limit
        public static final double kOutputLimit = 0.3;
    }
}
