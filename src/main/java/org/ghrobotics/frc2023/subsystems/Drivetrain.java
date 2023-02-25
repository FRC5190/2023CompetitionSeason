// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.controller.RamseteController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.CANSparkMax.ControlType;


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
    private final RamseteController ramsete_controller_;

    //Control
    private final SparkMaxPIDController left_pid_controller_;
    private final SparkMaxPIDController right_pid_controller_;
    private final DifferentialDriveFeedforward ff_;

    //Output Limit
    private boolean limit_output = false;

    //IO
    private OutputType output_type_ = OutputType.PERCENT;
    private final PeriodicIO io_ = new PeriodicIO();

    public Drivetrain(){
        //Initialize motor controllers
        left_leader_ = new CANSparkMax(Constants.kLeftLeaderId, MotorType.kBrushless);
        left_leader_.restoreFactoryDefaults();
        left_leader_.setIdleMode(IdleMode.kBrake);
        left_leader_.enableVoltageCompensation(12);
        left_leader_.setInverted(true);

        left_follower_ = new CANSparkMax(Constants.kLeftFollowerId, MotorType.kBrushless);
        left_follower_.restoreFactoryDefaults();
        left_follower_.setIdleMode(IdleMode.kBrake);
        left_follower_.enableVoltageCompensation(12);
        left_follower_.follow(left_leader_);

        right_leader_ = new CANSparkMax(Constants.kRightLeaderId, MotorType.kBrushless);
        right_leader_.restoreFactoryDefaults();
        right_leader_.setIdleMode(IdleMode.kBrake);
        right_leader_.enableVoltageCompensation(12);
        right_leader_.setInverted(false);

        right_follower_ = new CANSparkMax(Constants.kRightFollowerId, MotorType.kBrushless);
        right_follower_.restoreFactoryDefaults();
        right_follower_.setIdleMode(IdleMode.kBrake);
        right_follower_.enableVoltageCompensation(12);
        right_follower_.follow(right_leader_);

        //Initialize encoders
        left_encoder_ = left_leader_.getEncoder();
        left_encoder_.setPositionConversionFactor(
            2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio);
        left_encoder_.setVelocityConversionFactor(
            2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio / 60);

        right_encoder_= right_leader_.getEncoder();
        right_encoder_.setPositionConversionFactor(
            2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio);
        right_encoder_.setVelocityConversionFactor(
            2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio / 60);

        // Initialize PID controllers.
        left_pid_controller_ = left_leader_.getPIDController();
        left_pid_controller_.setP(Constants.kLeftKp);

        right_pid_controller_ = right_leader_.getPIDController();
        right_pid_controller_.setP(Constants.kRightKp);

         // Initialize feedforward.
        ff_ = new DifferentialDriveFeedforward(
            Constants.kLinearKv, Constants.kLinearKa, Constants.kAngularKv, Constants.kAngularKa
        );

        //Initialize Trajectory Tracking
        kinematics_ = new DifferentialDriveKinematics(Constants.kTrackWidth);
        ramsete_controller_ = new RamseteController();
    }

    @Override
    public void periodic(){
        //Read inputs
        io_.l_position = left_encoder_.getPosition();
        io_.r_position = right_encoder_.getPosition();
        io_.l_velocity = left_encoder_.getVelocity();
        io_.r_velocity = right_encoder_.getVelocity();

        io_.avg_velocity = (io_.l_velocity +  io_.r_velocity / 2);

        switch (output_type_){
            case PERCENT:
                left_leader_.set(limit_output ?
                    MathUtil.clamp(io_.l_demand, -Constants.kOutputLimit, Constants.kOutputLimit) :
                    io_.l_demand);
                right_leader_.set(limit_output ?
                    MathUtil.clamp(io_.r_demand, -Constants.kOutputLimit, Constants.kOutputLimit) :
                    io_.r_demand);
                break;
            case VELOCITY:
                // Calculate feedforward value and add to built-in motor controller PID.
                var wheel_voltages = ff_.calculate(
                    io_.l_velocity, io_.l_demand, io_.r_velocity, io_.r_demand, 0.02);

                left_pid_controller_.setReference(io_.l_demand, ControlType.kVelocity, 0,
                    wheel_voltages.left);
                right_pid_controller_.setReference(io_.r_demand, ControlType.kVelocity, 0,
                    wheel_voltages.right);
            break;
        }
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

    public void setVelocity(double l, double r) {
        output_type_ = OutputType.VELOCITY;
        io_.l_demand = l;
        io_.r_demand = r;
      }

    public double getLeftPosition(){
        return io_.l_position;
    }

    public double getRightPosition(){
        return io_.r_position;
    }

    public double getLeftVelocity() {
        return io_.l_velocity;
    }

    public double getRightVelocity() {
        return io_.r_velocity;
    }

    public double getVelocity() {
        return io_.avg_velocity;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics_;
    }

    public RamseteController getRamseteController() {
        return ramsete_controller_;
    }

    public void updateRobotPose(double LPosition, double RPosition, double angle){
        io_.l_position = LPosition;
        io_.r_position = RPosition;
        //io_.angle = angle;
    }


    enum OutputType {
        PERCENT, VELOCITY
      }

    public static class PeriodicIO {
        //Inputs
        double l_position;
        double r_position;
        double l_velocity;
        double r_velocity;
        double avg_velocity;


        //Outputs
        double l_demand;
        double r_demand;
    }

    public static class Constants {
        //Motor Controller IDs
        public static final int kLeftLeaderId = 1;
        public static final int kLeftFollowerId = 2;
        public static final int kRightLeaderId = 3;
        public static final int kRightFollowerId = 4;

        //Hardware
        public static double kGearRatio = 10.18;
        public static double kWheelRadius = 0.0762;
        public static double kTrackWidth = 0.759; //CHANGE
        //public static double kMass = 65.0; //CHANGE
        //public static double kMOI = 10.0; //CHANGE

        // Control
        public static final double kLinearKv = 2.6221;
        public static final double kLinearKa = 0.29686;
        public static final double kAngularKv = 2.9518;
        public static final double kAngularKa = 0.30714;

        public static final double kLeftKp = 0.0001;
        public static final double kRightKp = 0.0001;

        //Output Limit
        public static final double kOutputLimit = 0.3;
    }
}
