package frc.robot.subsystems.autonomous;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Config;

/**
 * Initial implementation of the Autonomous mode behavior
 */
public class AutonomousControllerImpl implements AutonomousController {

    private static AutonomousControllerImpl controller;

    private AutonomousControllerImpl(Config config, PathPlannableSubsystem driveSystem) {
        // Boolean supplier that controls when the path will be mirrored for the red
        // alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        BooleanSupplier requiresFlip = driveSystem::isRedAlliance;

        // AutoBuilder.configure(
        //         driveSystem::getPose,
        //         driveSystem::resetPose,
        //         driveSystem::getRobotRelativeChassisSpeeds,
        //         driveSystem::driveRobotRelative,
        //         driveSystem.getPathFollowingController(),
        //         config.generatedConfig,
        //         requiresFlip,
        //         driveSystem);
    }

    public static synchronized AutonomousControllerImpl initialize(Config config, PathPlannableSubsystem driveSystem) {
        controller = new AutonomousControllerImpl(config, driveSystem);
        return controller;
    }

    public static synchronized AutonomousControllerImpl instance() {
        if (controller == null) {
            throw new IllegalStateException("AutoController not initialized");
        }
        return controller;
    }

    @Override
    public void runInit() {
        Commands.print("No autonomous init configured").schedule();
    }

    @Override
    public void runPeriodic() {
        Commands.print("No autonomous periodic commands configured").schedule();
    }

    @Override
    public void runExit() {
        Commands.print("No autonomous exit configured").schedule();
    }
}
