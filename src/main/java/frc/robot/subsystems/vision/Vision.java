// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class Vision extends SubsystemBase {

  VisionIO[] io;
  VisionIOInputs[] inputs;
  VisionConsumer consumer;

  public Vision(VisionConsumer consumer, VisionIO ...io) {
    this.io = io;
    this.consumer = consumer;

    inputs = new VisionIOInputs[io.length];

    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputs();
    }
  }

  @Override
  public void periodic() {

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
    }

    for(int i = 0; i < io.length; i++) {
      for(PoseObservation observation : inputs[i].poseObservations) {
        double[] stdDevs = {0};
        consumer.acceptVisionMeasurement(observation.estimatedPose(), observation.timestamp(), new Matrix<N3, N1>(new SimpleMatrix(stdDevs)));
      }
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void acceptVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
