package org.ghrobotics.frc2023.subsystems;


import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase{
    public int moduleNumber;
    
    //Motor Controllers
    private final CANSparkMax drive_;
    private final CANSparkMax steer_;

    //Sensors
    private final RelativeEncoder drive_encoder_;
    private final RelativeEncoder steer_encoder_;
    private final CANCoder cancoder_;

    // Control
    private final PIDController drive_pid_controller_;
    private final PIDController position_pid_controller_;

    public SwerveModule(int driveMotorID, int steerMotorID, int CanCoderID) {
        // Initialize motor controllers
        drive_ = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        drive_.restoreFactoryDefaults();
        drive_.setIdleMode(IdleMode.kBrake);
        drive_.enableVoltageCompensation(12);
        drive_.setInverted(true);

        steer_ = new CANSparkMax(steerMotorID, MotorType.kBrushless);
        steer_.restoreFactoryDefaults();
        steer_.setIdleMode(IdleMode.kBrake);
        steer_.enableVoltageCompensation(12);
        steer_.setInverted(false);

        // Initialize encoders
        drive_encoder_ = drive_.getEncoder();
        drive_encoder_.setPositionConversionFactor(
            2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio);
        drive_encoder_.setVelocityConversionFactor(
            2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio / 60);

        steer_encoder_ = steer_.getEncoder();
        steer_encoder_.setPositionConversionFactor(
            2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio);
        steer_encoder_.setVelocityConversionFactor(
            2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio / 60);

        cancoder_ = new CANCoder(CanCoderID);
        CANCoderConfiguration config = new CANCoderConfiguration();
        // set units of the CANCoder to radians, with velocity being radians per second
        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoder_.configAllSettings(config);
    
        // Initialize PID controllers.
        drive_pid_controller_ = new PIDController (Constants.kDriveKp, Constants.kI, Constants.kD);
        position_pid_controller_ = new PIDController (Constants.kSteerKp, Constants.kI, Constants.kD);
        position_pid_controller_.enableContinuousInput (-Math.PI, Math.PI);
    }

    //getMethods
    public double DrivePosition(){
        return drive_encoder_.getPosition();
    }

    public double getDriveVelocity(){
        return drive_encoder_.getVelocity();
    }
    public double getSteerPosition(){
        return steer_encoder_.getPosition();
    }

    public double getSteerVelocity(){
        return steer_encoder_.getVelocity();
    }

    public double getCanCoderPosition(){
        return cancoder_.getAbsolutePosition();
    }

    //Setting the state of the module
    public void setDesiredState (SwerveModuleState desiredState){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState);
    }
    
    //Sets angle
    public void setAngle (SwerveModuleState desiredState){
        steer_.set(position_pid_controller_.calculate(getCanCoderPosition(), desiredState.angle.getRadians()));
    }

    //Sets speed
    public void setSpeed(SwerveModuleState desiredState){
        if (Math.abs(desiredState.speedMetersPerSecond) <0.001) {
            stop();
            return;
        }
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        double desiredDrive = desiredState.speedMetersPerSecond / Constants.kMaxSpeed;
        drive_.set(drive_pid_controller_.calculate(getDriveVelocity(), desiredDrive));
    }

    public void stop(){
        drive_.set(0);
        steer_.set(0); 
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getCanCoderPosition()));
    }

    //Does this get module position (distance and angle) from each module?
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDriveVelocity(), new Rotation2d(getSteerPosition()));
      }

    public static class Constants {

        // Hardware
        public static double kGearRatio = 0.0;
        public static double kWheelRadius = 0.0;
        public static double kTrackWidth = 0.0;

        public static final double kKs = 0.0;
        public static final double kKv = 0.0;
        public static final double kKa = 0.0;

        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kDriveKp = 0.0;
        public static final double kSteerKp = 0.0;

        // Output Limit
        public static final double kOutputLimit = 0.0;

        public static final double kMaxSpeed = 0.8;
  }
    
}

