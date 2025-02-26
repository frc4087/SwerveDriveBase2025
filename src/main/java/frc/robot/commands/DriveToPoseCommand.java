package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriveSpecs;
import frc.robot.subsystems.PPSwerveSubsystem;

/**
 * Commands the robot to drive from its current pose to a new pose at a given
 * speed. The nature of the command does not lend itself to chaining, as in a
 * path. Use PathPlanner instead. As such, zeros the speed at the beginning and
 * end of the command, but does not reset pose. As needed, use separate commands
 * before or after this call to reset the pose (e.g. ZeroPose).
 * <p>
 * Makes generous assumptions about error tolerance and PID tuning.
 * <p>
 * TODO: Delegate profile and controller building to the drive since it best
 * knows its needs and constraints.
 */
public class DriveToPoseCommand extends Command {
    /**
     * Creates an instance.
     * 
     * @param drivetrain  The target drivetrain.
     * @param speedFactor Speed factor relative to max [0, +1]. Sign is ignored.
     * @param pose        New pose.
     */
    public DriveToPoseCommand(PPSwerveSubsystem drive, Pose2d pose,
            double speedFactor) {
        _drive = drive;
        addRequirements(drive);

        _poseEnd = pose;
        _speedFactor = speedFactor;

        // build controller
        // TODO: Consider using profiles for X and Y
        Pose2d tolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(3.0));

        PIDController pidX = new PIDController(25.0, 0.0, 0.3);
        PIDController pidY = new PIDController(25.0, 0.0, 0.3);
        ProfiledPIDController pidR = new ProfiledPIDController(10.0, 0.0, 0.3,
                new TrapezoidProfile.Constraints(_drive.getSpecs().kMaxAngularVelRps,
                        _drive.getSpecs().kMaxAngularAccRpss));
        pidR.enableContinuousInput(-Math.PI, +Math.PI);

        _holoPid = new HolonomicDriveController(pidX, pidY, pidR);
        _holoPid.setTolerance(tolerance);

        reportInit();
    }

    @Override
    public void initialize() {
        // do nothing, don't stop
    }

    @Override
    public void execute() {
        Pose2d poseNow = _drive.getPose();
        ChassisSpeeds speeds = _holoPid.calculate(poseNow, _poseEnd,
                0.0, _poseEnd.getRotation());
        speeds.times(_speedFactor);
        ///// reportExecute(speeds);
        _drive.setDriveSpeeds(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        _drive.stop();
    }

    @Override
    public boolean isFinished() {
        boolean isDone = _holoPid.atReference();
        if (isDone)
            reportDone();
        return isDone;
    }

    // personal

    private void reportInit() {
        Pose2d poseNow = _drive.getPose();

        System.out.printf("DriveToPoseCommand.init: poseEnd[%6.2f %6.2f %6.1f], poseNow[%6.2f %6.2f %6.1f]\n",
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

        System.out.printf("DriveToPoseCommand.done: poseErr[%6.2f %6.2f %6.1f], poseEnd[%6.2f %6.2f %6.1f]\n",
                poseErr.getX(), poseErr.getY(), poseErr.getRotation().getDegrees(),
                _poseEnd.getX(), _poseEnd.getY(), _poseEnd.getRotation().getDegrees());
    }

    private final PPSwerveSubsystem _drive;
    private final Pose2d _poseEnd;
    private final double _speedFactor;
    private final HolonomicDriveController _holoPid;
}
