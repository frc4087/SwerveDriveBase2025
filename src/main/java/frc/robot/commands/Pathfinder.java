package frc.robot.commands;

import java.io.ObjectInputFilter.Config;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Pathfinder {

    private Pose2d targetPose;
    private PathConstraints constraints;

    private Map<String, Integer> tagIndices = new HashMap<String, Integer>();

    private void createTagMap() {
        tagIndices.put("Reef Front Right", 17);
        tagIndices.put("Reef Front", 18);
        tagIndices.put("Reef Front Left", 19);
        tagIndices.put("Reef Back Left", 20);
        tagIndices.put("Reef Back", 21);
        tagIndices.put("Reef Back Right", 22);
    }

    public Pathfinder() {

        constraints = new PathConstraints(
            3.0, 
            3.0,
            Units.degreesToRadians(540), 
            Units.degreesToRadians(720)
        );

        createTagMap();
    }

    public Pose2d getOffsetPose(String poseName) {

        Pose2d tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
            .getTagPose(tagIndices.get(poseName)).get().toPose2d();
        
        return tagPose.plus(
            new Transform2d(
                new Translation2d(2.0, 0.0).rotateBy(tagPose.getRotation()), tagPose.getRotation()
            )
        );
    }

    public Command getPathfindingCommand(String poseName) {
        targetPose = getOffsetPose(poseName);
        System.out.println("coding is fun");
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

}