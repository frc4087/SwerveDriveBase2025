package frc.robot.subsystems.teleop;
import edu.wpi.first.wpilibj.IterativeRobotBase;

/**
 * Describes the high-level behavior required to controll a robot's
 * teleop mode.
 */
public interface TeleopController {

    /**
     * Initialization code for teleop mode. Called each time the robot enters 
     * teleop mode.
     * 
     * See {@link IterativeRobotBase#teleopInit()}
     */
    void runInit();

    /**
     * Periodically called during teleop mode.
     * 
     * TODO: Look into what determines the period.
     * 
     * See {@link IterativeRobotBase#teleopPeriodic()}
     */
    void runPeriodic();

    /**
     * Exit code for teleop mode. Called each time the robot exits 
     * teleop mode.
     * 
     * This method clears all commands created in teleop mode in
     * preparation for disabled.
     * 
     * See {@link IterativeRobotBase#teleopExit()}
     */
    void runExit();
}
