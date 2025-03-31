package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriveSpecs;

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
    public PathToPoseCommand(CommandSwerveDrivetrain drive, Pose2d poseEnd,
            double speedFactor) {
        _drive = drive;
        addRequirements(_drive);

        _poseEnd = poseEnd;
        _speedFactor = Math.abs(speedFactor);
    }

    @Override
    public void initialize() {
        DriveSpecs specs = _drive.getConfig().getSpecs();
        PathConstraints ppSpecs = new PathConstraints(
                _speedFactor*specs.kMaxLinearVelMps(),
                _speedFactor*specs.kMaxLinearAccMpss(), 
                _speedFactor*specs.kMaxAngularVelRps(), 
                _speedFactor*specs.kMaxAngularAccRpss());
        _pathCommand = AutoBuilder.pathfindToPose(_poseEnd, ppSpecs);
        _pathCommand.initialize();
        reportInit();
    }

    @Override
    public void execute() {
        _pathCommand.execute();
        ///// reportExecute(_drive.getTrueSpeeds());
    }

    @Override
    public void end(boolean interrupted) {
        _pathCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        boolean isDone = _pathCommand.isFinished();
        if (isDone) {
            reportDone();
        }
        return isDone;
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

    private final CommandSwerveDrivetrain _drive;
    private final Pose2d _poseEnd;
    private final double _speedFactor;

    private Command _pathCommand;
}