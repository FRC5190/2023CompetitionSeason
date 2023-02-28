// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
//import org.json.*;

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
        io_.cl = table_.getEntry("cl").getDouble(0);
        io_.jsonString = table_.getEntry("json").getString("");

//        System.out.println(io_.jsonString);
//        System.out.println("Inside limelight periodic");


/*
    String jsonString = io_.jsonString ; //assign your JSON String here
    JSONObject obj = new JSONObject(jsonString);
    String pageName = obj.getJSONObject("Results");

    JSONArray arr = obj.getJSONArray("Fiducial"); // notice that `"posts": [...]`
    for (int i = 0; i < arr.length(); i++)
    {
        String post_id = arr.getJSONObject(i).getString("fID");
    }*/

        io_.tid = table_.getEntry("tid").getDouble(0);
        io_.botpose = table_.getEntry("botpose").getDoubleArray(new double[6]);
        io_.botpose_wpiblue = table_.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        io_.botpose_wpired = table_.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        io_.camerapose_targetspace = table_.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        io_.targetpose_cameraspace = table_.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

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
        return io_.tl;
    }

    public double getCaptureLatency(){
        return io_.cl;
    }

    public double getID() {
        return io_.tid;
    }


    public int getFID() {
        return io_.fid;
    }

    public double[] getbotpose() {
        return io_.botpose;
    }

    public double[] getBlueBotPose(){
        return io_.botpose_wpiblue;
    }

    public double[] getCamPosTargetSpace(){
        return io_.camerapose_targetspace;
    }

    public double[] getTargetPosCamSpace(){
        return io_.targetpose_cameraspace;
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
        double cl;
        String jsonString;
        int fid;
        double tid;
        double[] botpose;
        double[] botpose_wpiblue;
        double[] botpose_wpired;
        double[] camerapose_targetspace;
        double[] targetpose_cameraspace;

        double led_mode;
        double pipeline;
    }

}
