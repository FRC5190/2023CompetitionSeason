package org.ghrobotics.frc2023.subsystems;

import org.ghrobotics.frc2023.Superstructure.SuperstructureState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    // Motor Controllers
    private final CANSparkMax leader_;
    private final CANSparkMax follower_;

    // Sensors
    private final RelativeEncoder encoder_;

    // Feedforward
    private final ElevatorFeedforward feedforward;

    // Feedback
    private final SparkMaxPIDController pid_controller_;

    public Elevator() {

        // Initialize motor controllers
        leader_ = new CANSparkMax(Constants.kLeaderId, MotorType.kBrushless);
        leader_.setInverted(false);
        // figure out if motor should be inverted

        follower_ = new CANSparkMax(Constants.kFollowerId, MotorType.kBrushless);
        follower_.follow(leader_);

        encoder_ = leader_.getEncoder();
        // add conversion factor based on gear ratio - needs to be fixed
        encoder_.setPositionConversionFactor(2 * Math.PI / Constants.kGearRatio);
        encoder_.setVelocityConversionFactor(2 * Math.PI / Constants.kGearRatio / 60);

        feedforward = new ElevatorFeedforward(Constants.kS, Constants.kG, Constants.kV, Constants.kA);
        pid_controller_ = leader_.getPIDController();
        pid_controller_.setP(Constants.kP);

    }

    public double getPosition() {
        return encoder_.getPosition();
    }

    public void setPosition(SuperstructureState state) {
        // positive if target above, negative if below
        double dist = getTargetPosition(state) - getPosition();

        double v = dist / 0.02;
        double a = (v - getVelocity()) / 0.02;
        double ff = feedforward.calculate(v, a);

        leader_.setVoltage(feedforward.calculate(v, a));

        pid_controller_.setReference(v, ControlType.kVelocity, 0, ff);
    }

    public double getVelocity() {
        return encoder_.getVelocity();
    }

    // public void setVelocity(double v) {
    //     leader_.set(MathUtil.clamp(v, -Constants.kOutputLimit, Constants.kOutputLimit));
    // }


    // find values
    public double getTargetPosition(SuperstructureState state) {
        switch (state) {
            case SCORE_HIGH:
                return 0.0;
            case SCORE_MID:
                return 0.0;
            case PICKUP_GROUND:
                return 0.0;
            case PICKUP_SUBSTATION:
                return 0.0;
            default:
                return 0.0;
        }
    }

    public static class Constants {
        public static final int kFollowerId = 0;
        public static final int kLeaderId = 0;

        public static final double kMinPosition = 0;
        public static final double kMaxPosition = 0;
        // change based on height

        // public static final double kOutputLimit = 0;

        // Gear ratio
        public static final double kGearRatio = 20;

        // Feedforward 
        public static final double kS = 0; // volts
        public static final double kG = 0; // volts
        public static final double kV = 0; // volts * sec / distance 
        public static final double kA = 0; // volts * sec^2 / distance

        // Feedback
        public static final double kP = 0;

    }
}

