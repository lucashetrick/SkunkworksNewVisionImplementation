// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance;

  public Drivetrain() {
  }

  public static Drivetrain getInstance() {

    if (instance == null) {
      instance = new Drivetrain();
    }

    return instance;
  }

  SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(null, null, null, null);
  Vision vision = new Vision(getInstance()::addVisionMeasurement, new VisionIOPhotonVision("Photon Camera", new Transform3d()));

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {

    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
  }
}
