package frc.robot.autonomous;

import frc.robot.Config;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.systems.autonomous.AutonomousController;
import frc.robot.systems.autonomous.AutonomousControllerImpl;
import static frc.robot.ConfigTests.testConfigPath;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static org.awaitility.Awaitility.await;

import java.util.concurrent.TimeUnit;

public class AutonomousControllerTests {
    
    private static AutonomousController auto;
    public static CommandSwerveDrivetrain drivetrain;

    private static Config cfg = new Config(testConfigPath);

    @BeforeAll
    static void setupAll() {
        CommandScheduler.getInstance().enable();
        CommandScheduler.getInstance().printWatchdogEpochs();
        CommandScheduler.getInstance().run();

        DriverStationJNI.observeUserProgramStarting();
        drivetrain = new CommandSwerveDrivetrain(
            cfg,
            TunerConstants.DrivetrainConstants, 
            TunerConstants.FrontLeft, 
            TunerConstants.FrontRight, 
            TunerConstants.BackLeft, 
            TunerConstants.BackRight
        );
        auto = AutonomousControllerImpl.initialize(cfg, drivetrain);

    }

    @BeforeEach
    void setupEach() {
    }

    @Test
    void testStraightPath() {
        var cmd = auto.getAuto("Config")
            .ignoringDisable(true)
            .finallyDo(() -> System.out.println("END"));
        CommandScheduler.getInstance().schedule(cmd);
        await().atMost(10, TimeUnit.SECONDS).until(() -> {
            CommandScheduler.getInstance().run();
            var runningCommand = drivetrain.getCurrentCommand();
            System.out.println(String.format("Test CMD: %s - %s - %s", CommandScheduler.getInstance().isScheduled(cmd), cmd.isScheduled(), cmd.isFinished()));
            if (runningCommand != null) {
                System.out.println(String.format("Running CMD: %s - %s - %s", runningCommand.getName(), runningCommand.getClass().getName(), runningCommand.isFinished()));
            }
            return false;
        });
    }

}
