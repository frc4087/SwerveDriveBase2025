package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSpecs;
import frc.robot.subsystems.PPSwerveSubsystem;

/**
 * Commands the robot to drive from its current pose to a given end pose at a
 * given speed using PathPlanner and its obstacle avoidance.
 */
public class PathToPoseCommand extends Command {
    /**
     * Creates an instance.
     * 
     * @param drive       The target drivetrain.
     * @param speedFactor Speed factor relative to max [0, +1]. Sign is ignored.
     * @param poseEnd     End pose.
     */
    public PathToPoseCommand(PPSwerveSubsystem drive, Pose2d poseEnd,
            double speedFactor) {
        _drive = drive;
        addRequirements(drive.getSubsystems());

        _poseEnd = poseEnd;
        _speedFactor = Math.abs(speedFactor);
    }

    @Override
    public void initialize() {
        PathPlannerPath path = newPathFromCurrentPose();
        
        if (path.numPoints() < 2) {
            // start and end poses are equal: nothing to do, avoid PP error
            cancel();
        } else {
            _pathCommand = AutoBuilder.followPath(path);
            _pathCommand.initialize();
        }

        /////reportInit();
    }

    @Override
    public void execute() {
        _pathCommand.execute();
        /////reportExecute(_drive.getTrueSpeeds());
    }

    @Override
    public void end(boolean interrupted) {
        _pathCommand.end(interrupted);
        _drive.stop();
    }

    @Override
    public boolean isFinished() {
        boolean isDone = _pathCommand.isFinished();
        if (isDone) {
            reportDone();
        }
        return isDone;
    }

    // personal

    private PathPlannerPath newPathFromCurrentPose() {
        Pose2d poseNow = _drive.getPose();
        DriveSpecs specs = _drive.getSpecs();
        PathConstraints ppSpecs = new PathConstraints(specs.kMaxLinearVelMps,
                specs.kMaxLinearAccMpss, specs.kMaxAngularVelRps, specs.kMaxAngularAccRpss);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poseNow, _poseEnd);
        PathPlannerPath path = new PathPlannerPath(waypoints, ppSpecs,
                null, new GoalEndState(0.0, _poseEnd.getRotation()));
        path.preventFlipping = true;

        return path;
    }

    private void reportInit() {
        Pose2d poseNow = _drive.getPose();

        System.out.printf("PathToPoseCommand.init: poseEnd[%6.2f %6.2f %6.1f], poseNow[%6.2f %6.2f %6.1f]\n",
                _poseEnd.getX(), _poseEnd.getY(), _poseEnd.getRotation().getDegrees(),
                poseNow.getX(), poseNow.getY(), poseNow.getRotation().getDegrees());
    }

    private void reportExecute(ChassisSpeeds speeds) {
        Pose2d poseNow = _drive.getPose();
        Pose2d poseErr = _poseEnd.relativeTo(poseNow);

        System.out.printf("    poseErr[%6.2f %6.2f %6.1f], poseSpd[%6.2f %6.2f %6.1f]\n",
                poseErr.getX(), poseErr.getY(), poseErr.getRotation().getDegrees(),
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, Math.toDegrees(speeds.omegaRadiansPerSecond));
    }

    private void reportDone() {
        Pose2d poseNow = _drive.getPose();
        Pose2d poseErr = _poseEnd.relativeTo(poseNow);

        System.out.printf("PathToPoseCommand.done: poseErr[%6.2f %6.2f %6.1f], poseEnd[%6.2f %6.2f %6.1f]\n",
                poseErr.getX(), poseErr.getY(), poseErr.getRotation().getDegrees(),
                _poseEnd.getX(), _poseEnd.getY(), _poseEnd.getRotation().getDegrees());
    }

    private final PPSwerveSubsystem _drive;
    private final Pose2d _poseEnd;
    private final double _speedFactor;

    private Command _pathCommand;

    // class

    /**
     * Returns true if the path being followed should be mirrored (i.e. for the
     * red alliance)
     * 
     * @return The state.
     */
    private static boolean isPathFlipped() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

}
