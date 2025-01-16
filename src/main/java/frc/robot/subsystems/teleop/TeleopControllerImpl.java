package frc.robot.subsystems.teleop;

import edu.wpi.first.wpilibj2.command.Commands;

public class TeleopControllerImpl implements TeleopController {
    private static TeleopControllerImpl controller;

    private TeleopControllerImpl() {}

    public static synchronized TeleopControllerImpl initialize() {
        controller = new TeleopControllerImpl();
        return controller;
    }

    private static synchronized TeleopControllerImpl instance() {
        if (controller == null) {
            throw new IllegalStateException("TeleopController not initialized");
        }
        return controller;
    }

    @Override
    public void runInit() {
        Commands.print("No teleop init configured").schedule();
    }

    @Override
    public void runPeriodic() {
        Commands.print("No teleop periodic commands configured").schedule();
    }

    @Override
    public void runExit() {
        Commands.print("No teleop exit configured").schedule();
    }
}
