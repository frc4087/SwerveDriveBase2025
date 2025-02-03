package frc.robot.systems.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;

public class AutoVision {
    private Timer _timer;
  private Vision _targetVision;
  private boolean autoRoutine;

  /** Creates a new AutoVisionCommand. */
  public AutoVision(Vision targetVision, Pose2d pose, Rotation2d directionToDrive) {

    this._targetVision = targetVision;
    this.autoRoutine = false;

    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(_targetVision);

    // addCommands( 
    //     RobotContainer.S_SWERVE.runOnce(() -> {RobotContainer.CommandSwerveDrivetrain.resetOdometry(pose);}),
    //     RobotContainer.S_SWERVE.runOnce(() -> {RobotContainer.S_SWERVE.resetOdometry(pose);}),
    //     RobotContainer.S_SWERVE.runOnce(() -> {RobotContainer.S_SWERVE.resetOdometry(pose);}),
    //     RobotContainer.S_SWERVE.runOnce(() -> {RobotContainer.S_SWERVE.resetOdometry(pose);}),
    //     new PresetShotLaunchSequence(Shot.SUBWOOFER), 
    //     new RobotOrientedTimedDrive(new Translation2d(0.5, directionToDrive), 5.0)
    //   );
  }
}

  // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     _targetVision.cameraLEDToggleOn();
//     this.autoRoutine = true;
//   }

  // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

  // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return this.autoRoutine;
//     //return false;
//   }
//}
