package frc.robot.subsystems.teleop;
import edu.wpi.first.wpilibj.IterativeRobotBase;

/**
 * Describes the high-level behavior required to controll a robot's
 * autonomous mode.
 */
public interface TeleopController {

    /**
     * Initialization code for autonomous mode. Called each time the robot enters 
     * autonomous mode.
     * 
     * See {@link IterativeRobotBase#teleopInit()}
     */
    void runInit();

    /**
     * Periodically called during autonomous mode.
     * 
     * TODO: Look into what determines the period.
     * 
     * See {@link IterativeRobotBase#teleopPeriodic()()}
     */
    void runPeriodic();

    /**
     * Exit code for autonomous mode. Called each time the robot exits 
     * autonomous mode.
     * 
     * This method clears all commands created in autonomous mode in
     * preparation for teleop.
     * 
     * See {@link IterativeRobotBase#teleopExit()}
     */
    void runExit();
}
