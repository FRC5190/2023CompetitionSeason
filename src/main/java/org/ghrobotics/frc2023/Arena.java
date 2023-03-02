// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2023;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;

// The Playing Field
public class Arena {
  // April Tag Field Layout
  private static AprilTagFieldLayout field_layout_ = null;

  // Transforms Relative to Tag
  private static final Transform3d kTagToLPosition = new Transform3d(
      new Translation3d(0.6096, -0.7366, -0.46272),
      new Rotation3d(0, 0, Math.PI));
  private static final Transform3d kTagToRPosition = new Transform3d(
      new Translation3d(0.6096, +0.7366, -0.46272),
      new Rotation3d(0, 0, Math.PI));
  private static final Transform3d kTagToCPosition = new Transform3d(
      new Translation3d(0.6096, 0, -0.46272),
      new Rotation3d(0, 0, Math.PI));

  // Initialization
  static {
    try {
      field_layout_ = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException ex) {
      System.out.println("An error occurred when loading the AprilTag field!");
    }
  }

  // Tag Position Getter
  public static Pose3d getTagPosition(int id) {
    return field_layout_.getTagPose(id).orElse(null);
  }

  // Tag Left Transform Getter
  public static Transform3d getTagToLPosition() {
    return kTagToLPosition;
  }

  // Tag Right Transform Getter
  public static Transform3d getTagToRPosition() {
    return kTagToRPosition;
  }

  // Tag Center Transform Getter
  public static Transform3d getTagToCPosition() {
    return kTagToCPosition;
  }
}
