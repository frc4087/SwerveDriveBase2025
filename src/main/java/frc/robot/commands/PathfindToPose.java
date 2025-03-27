package frc.robot.commands;

import java.util.Map;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

import frc.robot.Config;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PathfindToPose extends Command {

    private final PIDController headingController;

    private Pose2d targetPose;
    private PathConstraints constraints;

    private Map<String, Integer> tagIndices = new HashMap<String, Integer>;;

    private void createTagMap() {
        tagIndices.put("Left Intake", 13);
        tagIndices.put("Right Intake", 12);
        tagIndices.put("Reef Front Right", 17);
        tagIndices.put("Reef Front", 18);
        tagIndices.put("Reef Front Left", 19);
        tagIndices.put("Reef Back Left", 20);
        tagIndices.put("Reef Back", 21);
        tagIndices.put("Reef Back Right", 22);
    }

    public PathfindToPose(CommandSwerveDrivetrain drivetrain) {
        super();
        this.addRequirements(drivetrain);

        constraints = new PathConstraints(3.0, 4.0, units.degreesToRadians(540), Units.degreesToRadians(720));

        createTagMap();
    }

    public PathfindToPose withPoseName(String name) {
        targetPose = new AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(tagIndices.get(name));
    }

    @Override
    public void execute() {
        AutoBuilder.PathfindToPose(
            targetPose,
            constraints,
            0.0,
            0.0
        ).execute();
    }
}