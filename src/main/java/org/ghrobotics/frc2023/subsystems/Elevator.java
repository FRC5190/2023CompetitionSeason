package org.ghrobotics.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.ghrobotics.frc2023.Superstructure.RobotState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    // use elevator feedforward

    // Motor Controllers
    private final CANSparkMax left_leader_;
    private final CANSparkMax left_follower_;
    private final CANSparkMax right_leader_;
    private final CANSparkMax right_follower_;

    // Sensors
    private final RelativeEncoder left_encoder_;
    private final RelativeEncoder right_encoder_;

    public Elevator() {

        // Initialize motor controllers
        left_leader_ = new CANSparkMax(Constants.kLeftLeaderId, MotorType.kBrushless);
        left_leader_.setInverted(false);
        // figure out which motor should be inverted

        left_follower_ = new CANSparkMax(Constants.kLeftFollowerId, MotorType.kBrushless);
        left_follower_.follow(left_leader_);

        right_leader_ = new CANSparkMax(Constants.kRightLeaderId, MotorType.kBrushless);
        right_leader_.setInverted(true);

        right_follower_ = new CANSparkMax(Constants.kRightFollowerId, MotorType.kBrushless);
        right_follower_.follow(right_leader_);

        left_encoder_ = left_leader_.getEncoder();
        // add conversion factor based on gear ratio

        right_encoder_= right_leader_.getEncoder();
        // add conversion factor based on gear ratio
    }

    public double getLeftPosition() {
        return left_encoder_.getPosition();
        // convert from encoder values to elevator position
    }

    public double getRightPosition() {
        return right_encoder_.getPosition();
    }

    public double getLeftVelocity() {
        return left_encoder_.getVelocity();
    }

    public double getRightVelocity() {
        return right_encoder_.getVelocity();
    }

    public void setVelocity(double l, double r) {
        left_leader_.set(MathUtil.clamp(l, -Constants.kOutputLimit, Constants.kOutputLimit));
        right_leader_.set(MathUtil.clamp(r, -Constants.kOutputLimit, Constants.kOutputLimit));
    }

    // find values
    public double getTargetPosition(RobotState state) {
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
        public static final int kRightFollowerId = 0;
        public static final int kRightLeaderId = 0;
        public static final int kLeftFollowerId = 0;
        public static final int kLeftLeaderId = 0;

        public static final double kMinPosition = 0;
        public static final double kMaxPosition = 0;
        // change based on height

        public static final double kOutputLimit = 0;
    }
}

