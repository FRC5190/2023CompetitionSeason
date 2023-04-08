package org.ghrobotics.frc2023.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private AHRS gyro_;
    private Pose2d pose = new Pose2d (0.0, 0.0, new Rotation2d());
    private SwerveModule[] swerveModules = new SwerveModule [] {
    new SwerveModule(Constants.kFLDriveMotor, Constants.kFLSteerMotor, Constants.kFLCanCoder),
    new SwerveModule(Constants.kBLDriveMotor, Constants.kBLSteerMotor, Constants.kBLCanCoder),
    new SwerveModule(Constants.kFRDriveMotor, Constants.kFRSteerMotor, Constants.kBRCanCoder),
    new SwerveModule(Constants.kBRDriveMotor, Constants.kBRSteerMotor, Constants.kBRCanCoder)
    };

    SwerveModule frontLeft = swerveModules [0];
    SwerveModule backLeft = swerveModules [1];
    SwerveModule frontRight = swerveModules [2];
    SwerveModule backRight = swerveModules [3];

    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
    
    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    public SwerveDriveOdometry odometry = new SwerveDriveOdometry (m_kinematics, gyro_.getRotation2d(), new SwerveModulePosition[] {
        frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition() }, pose);

    @Override
    public void periodic() {
        // Get the rotation of the robot from the gyro.
        var gyroAngle = gyro_.getRotation2d();

        // Update the pose
        pose = odometry.update((Rotation2d) gyroAngle,
            new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()
            });
    }

    public Drive(){

       gyro_ = new AHRS(SPI.Port.kMXP);
       
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
        } catch (Exception e) {
        }
        }).start();
    }

    public void zeroHeading(){
        gyro_.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro_.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition [] positions = new SwerveModulePosition[4];
        for (SwerveModule mod: swerveModules){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.maxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    public static class Constants{
        //Drive Motors
        public static final int kFLDriveMotor = 0;
        public static final int kFRDriveMotor = 0;
        public static final int kBLDriveMotor = 0;
        public static final int kBRDriveMotor = 0;

        //Steer Motors
        public static final int kFLSteerMotor = 0;
        public static final int kFRSteerMotor = 0;
        public static final int kBLSteerMotor = 0;
        public static final int kBRSteerMotor = 0;

        //CanCodrs
        public static final int kFLCanCoder = 0;
        public static final int kFRCanCoder = 0;
        public static final int kBLCanCoder = 0;
        public static final int kBRCanCoder = 0;

        public static final double maxSpeed = 0.5;
    }
    
}
