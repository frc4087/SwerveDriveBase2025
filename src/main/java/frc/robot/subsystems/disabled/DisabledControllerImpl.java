package frc.robot.subsystems.disabled;

import edu.wpi.first.wpilibj2.command.Commands;

public class DisabledControllerImpl implements DisabledController {
    private static DisabledControllerImpl controller;

    private DisabledControllerImpl() {}

    public static synchronized DisabledControllerImpl initialize() {
        controller = new DisabledControllerImpl();
        return controller;
    }
    
    public static synchronized DisabledControllerImpl instance() {
        if (controller == null) {
            throw new IllegalStateException("DisabledController not initialized");
        }
        return controller;
    }

    @Override
    public void runInit() {
        Commands.print("No disabled init configured").schedule();
    }

    @Override
    public void runPeriodic() {
        Commands.print("No disabled periodic commands configured").schedule();
    }

    @Override
    public void runExit() {
        Commands.print("No disabled exit configured").schedule();
    }
}
