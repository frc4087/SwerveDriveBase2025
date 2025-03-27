package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSpecs;

public class FieldPoses {

    public FieldPoses(DriveSpecs specs) {
        AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        // All is per the field drawing
        // (https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf)
        // and relative to the view from the blue alliance.

        poseLoaderLeft = offsetFieldPose(tags.getTagPose(13).get().toPose2d(), specs.kFrontOffsetM());
        poseLoaderRight = offsetFieldPose(tags.getTagPose(12).get().toPose2d(), specs.kFrontOffsetM());

        poseReefFrontLeft = offsetFieldPose(tags.getTagPose(19).get().toPose2d(), specs.kFrontOffsetM());
        poseReefFrontCenter = offsetFieldPose(tags.getTagPose(18).get().toPose2d(), specs.kFrontOffsetM());
        poseReefFrontRight = offsetFieldPose(tags.getTagPose(17).get().toPose2d(), specs.kFrontOffsetM());

        poseReefBackLeft = offsetFieldPose(tags.getTagPose(20).get().toPose2d(), specs.kFrontOffsetM());
        poseReefBackCenter = offsetFieldPose(tags.getTagPose(21).get().toPose2d(), specs.kFrontOffsetM());
        poseReefBackRight = offsetFieldPose(tags.getTagPose(22).get().toPose2d(), specs.kFrontOffsetM());
    }

    // class

    public final Pose2d poseLoaderLeft;
    public final Pose2d poseLoaderRight;
    public final Pose2d poseReefFrontLeft;
    public final Pose2d poseReefFrontCenter;
    public final Pose2d poseReefFrontRight;
    public final Pose2d poseReefBackLeft;
    public final Pose2d poseReefBackCenter;
    public final Pose2d poseReefBackRight;

    /**
     * Returns a field pose that is offset by a given amount from a given input
     * field pose in the direction of the pose. The field coordinate system is
     * assumed right-handed with Z up.
     * 
     * @param poseIn Input pose.
     * @param offset Offset (m) relative to the input pose direction (positive
     *               offset is "forward").
     * @return Output pose.
     */
    private static Pose2d offsetFieldPose(Pose2d poseIn, double offset) {
        double resultX = poseIn.getX() + offset * poseIn.getRotation().getCos();
        double resultY = poseIn.getY() + offset * poseIn.getRotation().getSin();
        return new Pose2d(resultX, resultY, poseIn.getRotation());
    }
}
