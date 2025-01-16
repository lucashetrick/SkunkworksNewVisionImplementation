// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionMeasurement;

public class Vision extends SubsystemBase {

  VisionIO[] io;
  VisionConsumer consumer;

  public Vision(VisionConsumer consumer, VisionIO ...io) {
    this.io = io;
    this.consumer = consumer;
  }

  @Override
  public void periodic() {
    for (VisionIO io : io) {
      VisionMeasurement data = io.getLatestData();
      for(PoseObservation observation : data.poseObservations) {
        consumer.accept(observation.estimatedPose(), observation.timestamp(), observation.stdDevs());
      }
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d estimatedPose,
        double timestamp,
        Matrix<N3, N1> stdDevs
      );
  }
}
