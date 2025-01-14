package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {

    record PoseObservation(
        double timestamp,
        Pose2d estimatedPose,
        double ambiguity) {
    }

    public static class VisionIOInputs {
        public PoseObservation[] poseObservations = new PoseObservation[0];
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
