package frc.robot.subsystems.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.DriveFeedforwards;

/**
 * Describes the behavior required for a {@link Subsystem} to work with
 * PathPlanner.
 * See {@link https://pathplanner.dev/pplib-build-an-auto.html#configure-autobuilder}
 * for more information.
 */
public interface PathPlannable extends Subsystem {

  /**
   * @return the current robot {@link Pose2d }. Units in meters.
   */
  Pose2d getPose();

  /**
   * Resets the robot's pose to the provided pose
   * 
   * @param current {@link Pose2d }. Units in meters.
   */
  void resetPose(Pose2d pose);

  /**
   * Returns the current robot-relative ChassisSpeeds.
   * 
   * @return current {@link ChassisSpeeds }
   */
  ChassisSpeeds getRobotRelativeChassisSpeeds();

  /**
   * Moves the robot based on the provided robot-relative {@link ChassisSpeeds}
   * and {@link DriveFeedforwards}. When using swerve, these feedforwards 
   * will be in FL, FR, BL, BR order.
   * 
   * @param robotRelativeSpeeds
   * @param driveFeedforwards
   */
  void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds, DriveFeedforwards driveFeedforwards);

  /**
   * Path following controller that will be used to follow paths.
   * 
   * @param a {@link PathFollowingController}
   */
  PathFollowingController getPathFollowingController();

  /**
   * Used in path following algorithms to determine if a positional flip is
   * required.
   * 
   * @return true if robot is part of the red alliance
   */
  boolean isRedAlliance();
}
