package frc.robot.subsystems.disabled;
import edu.wpi.first.wpilibj.IterativeRobotBase;

/**
 * Describes the high-level behavior required to controll a robot's
 * disabled mode.
 */
public interface DisabledController {

    /**
     * Initialization code for disabled mode. Called each time the robot enters 
     * disabled mode.
     * 
     * See {@link IterativeRobotBase#disabledInit()}
     */
    void runInit();

    /**
     * Periodically called during disabled mode.
     * 
     * TODO: Look into what determines the period.
     * 
     * See {@link IterativeRobotBase#disabledPeriodic()}
     */
    void runPeriodic();

    /**
     * Exit code for disabled mode. Called each time the robot exits 
     * disabled mode.
     * 
     * This method clears all commands created in disabled mode in
     * preparation for teleop.
     * 
     * See {@link IterativeRobotBase#disabledExit()}
     */
    void runExit();
}
