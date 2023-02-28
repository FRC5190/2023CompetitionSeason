// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import java.io.IOException;

/** Add your docs here. */
public class Arena {

    public AprilTagFieldLayout blueLayout;

    public static final Pose3d[] tagPositions = new Pose3d[]{
        new Pose3d(15.513, 1.0734, 0.46272, null), //1
        new Pose3d(15.513, 2.7498, 0.46272, null), //2
        new Pose3d(15.513, 4.4262, 0.46272, null), //3
        new Pose3d(1.0269, 4.4262, 0.46272, null), //6
        new Pose3d(1.0269, 2.7498, 0.46272, null), //7
        new Pose3d(1.0269, 1.0734, 0.46272, null), //8
    };

   public static final Transform3d[] blueTransform = {
                    new Transform3d(new Translation3d(0.6096, -0.7366, -0.46272), new Rotation3d()), //Left
                    new Transform3d(new Translation3d(0, 0, 0), new Rotation3d()), //Center
                    new Transform3d(new Translation3d(0, 0, 0), new Rotation3d()),//Right
                    }; 

    public static final Transform3d[] redTransform = {
        new Transform3d(new Translation3d(-0.6096, 0.7366, -0.46272), new Rotation3d()), //Left
        new Transform3d(new Translation3d(0, 0, 0), new Rotation3d()), //Center
        new Transform3d(new Translation3d(0, 0, 0), new Rotation3d()),//Right
        }; 
    public static final Transform3d[] highAndMidTransform = {
        new Transform3d(new Translation3d(0.554, 0.600, 0.675), new Rotation3d()), //High
        new Transform3d(new Translation3d(0.270, 0.170, 0.270), new Rotation3d()) //Mid
    };

    public Arena(){
        try{
            blueLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        }
        catch(IOException ex) {
            System.out.println("Error reading tag layout");
        }

    }
                    
    

//Layout of Grids ( A | B | C)
/* A 
 0 | 1 | 2 
 3 | 4 | 5
 6 | 7 | 8
*/


}
