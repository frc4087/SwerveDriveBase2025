package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveSpecs;

public class FieldPoses {

    public final Pose2d poseLoaderLeft;
    public final Pose2d poseLoaderRight;
    public final Pose2d poseReefFrontLeft;
    public final Pose2d poseReefFrontCenter;
    public final Pose2d poseReefFrontRight;
    public final Pose2d poseReefBackLeft;
    public final Pose2d poseReefBackCenter;
    public final Pose2d poseReefBackRight;

    public FieldPoses(DriveSpecs specs) {
        AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        // All is per the field drawing
        // (https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf)
        // and relative to the view from the blue alliance.

        poseLoaderLeft = offsetFieldPose(tags.getTagPose(13).get().toPose2d(),
                Rotation2d.fromDegrees(180.0), new Translation2d(-specs.kFrontOffsetM(), 0.0));
        poseLoaderRight = offsetFieldPose(tags.getTagPose(12).get().toPose2d(),
                Rotation2d.fromDegrees(180.0), new Translation2d(-specs.kFrontOffsetM(), 0.0));

        poseReefFrontLeft = offsetFieldPose(tags.getTagPose(19).get().toPose2d(),
                Rotation2d.fromDegrees(180.0), new Translation2d(-specs.kFrontOffsetM(), 0.0));
        poseReefFrontCenter = offsetFieldPose(tags.getTagPose(18).get().toPose2d(),
                Rotation2d.fromDegrees(180.0), new Translation2d(-specs.kFrontOffsetM(), 0.0));
        poseReefFrontRight = offsetFieldPose(tags.getTagPose(17).get().toPose2d(),
                Rotation2d.fromDegrees(180.0), new Translation2d(-specs.kFrontOffsetM(), 0.0));

        poseReefBackLeft = offsetFieldPose(tags.getTagPose(20).get().toPose2d(),
                Rotation2d.fromDegrees(180.0), new Translation2d(-specs.kFrontOffsetM(), 0.0));
        poseReefBackCenter = offsetFieldPose(tags.getTagPose(21).get().toPose2d(),
                Rotation2d.fromDegrees(180.0), new Translation2d(-specs.kFrontOffsetM(), 0.0));
        poseReefBackRight = offsetFieldPose(tags.getTagPose(22).get().toPose2d(),
                Rotation2d.fromDegrees(180.0), new Translation2d(-specs.kFrontOffsetM(), 0.0));
    }

    // class

    /**
     * Returns a field pose that is offset by a given amount from a given input
     * field pose in the direction of the pose. The field coordinate system is
     * assumed right-handed with Z up.
     * 
     * @param poseIn  Input pose (e.g. AprilTag).
     * @param rotateZ CCW rotation (deg) relative to the input pose direction.
     * @param xlateXY XY offset (m) relative to the input pose position and output
     *                pose direction (+X is forward, +Y left).
     * @return Output pose.
     */
    private static Pose2d offsetFieldPose(Pose2d poseIn, Rotation2d rotateZ, Translation2d xlateXY) {
        Rotation2d rotOut = poseIn.getRotation().plus(rotateZ);
        double resultX = poseIn.getX() + xlateXY.getX() * rotOut.getCos() + xlateXY.getY() * rotOut.getSin();
        double resultY = poseIn.getY() + xlateXY.getX() * rotOut.getSin() + xlateXY.getY() * rotOut.getCos();
        return new Pose2d(resultX, resultY, rotOut);
    }
}
