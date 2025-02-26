package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Interface for a drivetrain (differential or holonomic) that supports
 * basic drive commands (e.g. dive or turn to a given absolute or
 * relative pose).
 */
public interface CommandDrivable extends Subsystem {

	/**
	 * Resets the current robot pose, which is presumed to be relative to the
	 * play field. Ideally, the pose is that of the actual robot with a known
	 * pose (e.g. at the start of competition or from subsequent sensor data).
	 * 
	 * @param pose The pose.
	 */
	void resetPose(Pose2d pose);

	/**
	 * Gets the current robot pose, which is presumed to be relative to the
	 * play field. In reality this pose will be relative to the most recent
	 * resetPose().
	 * 
	 * @return The pose.
	 */
	Pose2d getPose();

	/**
	 * Sets the desired robot relative velocities. Differential drives should
	 * should ignore non-zero Y velocity.
	 * 
	 * @param speeds The velocities.
	 */
	void setDriveSpeeds(ChassisSpeeds speeds);

	/**
	 * Gets the maximum robot relative positive velocities (i.e. assumed
	 * independent of direction). Used for converting between
	 * speed factors and velocities. Differential drives should
	 * have zero Y velocity.
	 * 
	 * @return The velocities.
	 */
	ChassisSpeeds getMaxSpeeds();

	/**
	 * Immediately stops any robot movement by setting chassis speeds to zero and
	 * resetting all PID controllers.
	 */
	void stop();

	/**
	 * Returns true if the drive is holonomic (e.g. swerve), otherwise assumes
	 * the drive is differential (i.e. no lateral movement).
	 * 
	 * @return The status.
	 */
	boolean isHolonomic();

	/**
	 * Gets the subsystems required to support this subsystem, including this
	 * one.
	 * 
	 * @return Temp output group.
	 */
	List<Subsystem> getSubsystems();
}