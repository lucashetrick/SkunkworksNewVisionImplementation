package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
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
    public void updateInputs(VisionIOInputs inputs) {
        List<PoseObservation> poseObservations = new LinkedList<PoseObservation>();
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

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
                        PoseObservation poseObservation = new PoseObservation(timestamp, pose2d, 0);
                        poseObservations.add(poseObservation);
                    }
                }
            }
        }

        inputs.poseObservations = new PoseObservation[poseObservations.size()];

        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }
    }
}