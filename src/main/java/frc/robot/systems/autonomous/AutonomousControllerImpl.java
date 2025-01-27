package frc.robot.systems.autonomous;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Config;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * Initial implementation of the Autonomous mode behavior
 */
public class AutonomousControllerImpl implements AutonomousController {

    private static AutonomousControllerImpl controller;

    private Map<String, PathPlannerAuto> autos;

    private final SendableChooser<Command> autoChooser;

    private Distance startingX;
    private Distance startingY;

    private AutonomousControllerImpl(Config config, PathPlannableSubsystem driveSystem) {
        // Boolean supplier that controls when the path will be mirrored for the red
        // alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        BooleanSupplier requiresFlip = driveSystem::isRedAlliance;

        AutoBuilder.configure(
            driveSystem::getPose,
            (Pose2d pose) -> {
                startingX = pose.getMeasureX();
                startingY = pose.getMeasureY();
                driveSystem.resetPose(pose);
            },
            driveSystem::getRobotRelativeChassisSpeeds,
            driveSystem::driveRobotRelative,
            driveSystem.getPathFollowingController(),
            config.generatedConfig,
            requiresFlip,
            driveSystem);
        loadAutos();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        this.startingX = driveSystem.getPose().getMeasureX();
        this.startingY = driveSystem.getPose().getMeasureY();
        Commands.print(String.format(
            "Starting At: (%s, %s)", 
            driveSystem.getPose().getMeasureX().minus(startingX),
            driveSystem.getPose().getMeasureY().minus(startingY)
        )).schedule();

    }

    private void loadAutos() {
        autos = Map.of(
            "Config", new PathPlannerAuto("Config")
        );
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
        CommandScheduler.getInstance().cancelAll();
        autos.get("Config").schedule();
    }

    @Override
    public void runPeriodic() {}

    @Override
    public void runExit() {
        Commands.print("No autonomous exit configured").schedule();
    }
}
