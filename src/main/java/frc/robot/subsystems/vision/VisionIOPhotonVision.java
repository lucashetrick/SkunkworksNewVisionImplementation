package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;

public class VisionIOPhotonVision implements VisionIO {

    private final PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator poseEstimator;

    public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);

        try {
            aprilTagFieldLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Exception loading AprilTag field layout JSON: " + e.toString());
        }

        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    }

    @Override
    public VisionMeasurement getLatestData() {
        List<PoseObservation> poseObservations = new LinkedList<PoseObservation>();
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        VisionMeasurement latest = new VisionMeasurement();

        if (!results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size() - 1);

            if (result.hasTargets()) {
                Optional<EstimatedRobotPose> updatedPose = poseEstimator.update(result);

                if (updatedPose.isPresent()) {
                    EstimatedRobotPose pose = updatedPose.get();

                    if (pose.estimatedPose.toPose2d().getX() > 0.0
                            && pose.estimatedPose.toPose2d().getX() < Constants.FIELD_X_LENGTH
                            && pose.estimatedPose.toPose2d().getY() > 0.0
                            && pose.estimatedPose.toPose2d().getY() < Constants.FIELD_Y_LENGTH) {

                        double timestamp = pose.timestampSeconds;
                        Pose2d pose2d = pose.estimatedPose.toPose2d();
                        double[] stdDevs = {0};
                        PoseObservation poseObservation = new PoseObservation(timestamp, pose2d, new Matrix<N3, N1>(new SimpleMatrix(stdDevs)));
                        poseObservations.add(poseObservation);
                    }
                }
            }
        }

        latest.poseObservations = new PoseObservation[poseObservations.size()];

        for (int i = 0; i < poseObservations.size(); i++) {
            latest.poseObservations[i] = poseObservations.get(i);
        }

        return latest;
    }
}
