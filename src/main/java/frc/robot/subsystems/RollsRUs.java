package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class RollsRUs extends SubsystemBase {
    private TalonFX intakeMotor;
    private Integer intakeSpeed;
    private Integer outputSpeed;

    public RollsRUs(Config config) {
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = config.readIntegerProperty("rollsRUs.motor.current.limit.amps");
        limitConfigs.StatorCurrentLimitEnable = true;

        intakeMotor.getConfigurator().apply(limitConfigs);

        intakeSpeed = config.readIntegerProperty("rollsRUs.motor.intake.speed");
        outputSpeed = config.readIntegerProperty("rollsRUs.motor.output.speed");
        
        var intakeMotorPort = config.readIntegerProperty("ports.intake.motor");
        intakeMotor= new TalonFX(intakeMotorPort);
    }

    public Command runIntake() {
        return this.runEnd(() -> intakeMotor.set(intakeSpeed), this::stop);
    }

    public Command runOutput() {
        return this.runEnd(() -> intakeMotor.set(outputSpeed), this::stop);
    }

    private void stop() {
        intakeMotor.set(0);
    }
}
