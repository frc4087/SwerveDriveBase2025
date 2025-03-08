package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Creates a subsystem that augments the target drive pose using LimeLight
 * MegaTag2 localization. Note: This subsystem MUST be included along with the
 * drive subsystem when incorporated into a command.
 */
public class LLPoseSubsystem extends SubsystemBase {
    public LLPoseSubsystem(CommandSwerveDrivetrain drive) {
        _drive = drive;
    }

    @Override
    public void periodic() {
        // update megatag rotation
        LimelightHelpers.SetRobotOrientation("limelight", _drive.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (mt2 == null) {
            // LimeLight is missing: report once, do nothing
            if (!_isMissingReported) {
                _isMissingReported = true;
                _isFoundReported = false;
                System.err.println("LLPoseSubsystem: WARNING - LimeLight camera is missing.");
            }

            return; // done
        } else {
            if (!_isFoundReported) {
                _isFoundReported = true;
                _isMissingReported = false;
                System.err.println("LLPoseSubsystem: LimeLight camera found.");
            }
        }

        // resolve data quality
        boolean isReject = false;
        if (Math.abs(_drive.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) {
            // gyro rate too high
            isReject = true;
        }
        if (mt2.tagCount == 0) {
            // no apriltags in sight
            isReject = true;
        }

        if (isReject) {
            // LimeLight data is bad: report once, do nothing
            if (!_isBadReported) {
                _isBadReported = true;
                _isGoodReported = false;
                System.err.println("LLPoseSubsystem: LimeLight camera data is BAD.");
            }

            return; // done
        } else {
            if (!_isGoodReported) {
                _isGoodReported = true;
                _isBadReported = false;
                System.err.println("LLPoseSubsystem: LimeLight camera data is GOOD.");
            }
        }

        // data is good, fuse vision pose with drive pose
        _drive.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        _drive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }

    // personal

    private final CommandSwerveDrivetrain _drive;
    private boolean _isMissingReported = false;
    private boolean _isFoundReported = false;
    private boolean _isBadReported = false;
    private boolean _isGoodReported = false;
}
