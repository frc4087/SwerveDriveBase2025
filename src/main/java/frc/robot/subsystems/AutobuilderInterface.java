package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface AutobuilderInterface {

    public Pose2d getPose(); //Returns the current robot pose as a Pose2d
    

    public Pose2d resetPose(); //Resets the robot's odometry to the given pose


  /*Returns the current robot-relative ChassisSpeeds. This can be calculated using one of WPILib's drive kinematics classes */
    public Pose2d getRobotRelativeSpeeds();

  /*Outputs commands to the robot's drive motors given robot-relative ChassisSpeeds. This can be converted to module states or wheel speeds using WPILib's drive kinematics classes */
    public Pose2d driveRobotRelative();


}
