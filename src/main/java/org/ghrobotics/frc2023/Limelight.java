// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class Limelight {
    private final NetworkTable table_;

    private final PeriodicIO io_ = new PeriodicIO();

    public Limelight(String name) {
        table_ = NetworkTableInstance.getDefault().getTable(name);
    }

    public void periodic() {
        io_.tv = table_.getEntry("tv").getDouble(0);
        io_.tx = table_.getEntry("tx").getDouble(0);
        io_.ty = table_.getEntry("ty").getDouble(0);
        io_.ta = table_.getEntry("ta").getDouble(0);
        io_.ts = table_.getEntry("ts").getDouble(0);
        io_.tl = table_.getEntry("tl").getDouble(0);

        table_.getEntry("ledMode").setDouble(io_.led_mode);
        table_.getEntry("pipeline").setDouble(io_.pipeline);
    }

    public boolean hasTarget() {
        return io_.tv != 0;
    }

    public double getTx() {
        return io_.tx;
    }

    public double getTy() {
        return io_.ty;
    }
 
    public double getArea() {
        return io_.ta;
    }

    public double getSkew() {
        return io_.ts;
    }

    public double getLatency() {
        return 11 + io_.tl;
    }

    public int getID() {
        return io_.tid;
    }

    public void setLED(LEDMode mode) {
        io_.led_mode = mode.ordinal();
    }

    public void setPipeline(int pipeline) {
        io_.pipeline = pipeline;
    }

    public enum LEDMode {
        PIPELINE, OFF, BLINK, ON
    }

    static class PeriodicIO {
        double tv;
        double tx;
        double ty;
        double ta;
        double ts;
        double tl;
        int tid;

        double led_mode;
        double pipeline;
    }

}
