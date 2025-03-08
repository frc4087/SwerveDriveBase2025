package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class RollsRUs extends SubsystemBase {
    private TalonFX intakeMotor;
    private Double intakeSpeed;
    private Double outputSpeed;

    public RollsRUs(Config config) {

        var intakeMotorPort = config.readIntegerProperty("ports.intake.motor");
        intakeMotor = new TalonFX(intakeMotorPort);
        
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = config.readIntegerProperty("rollsRUs.motor.statorCurrent.limit.amps");
        limitConfigs.StatorCurrentLimitEnable = true;

        limitConfigs.SupplyCurrentLimit = config.readIntegerProperty("rollsRUs.motor.supplyCurrent.limit.amps");
        limitConfigs.SupplyCurrentLimitEnable = true;

        intakeMotor.getConfigurator().apply(limitConfigs);

        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        intakeSpeed = config.readDoubleProperty("rollsRUs.motor.intake.speed");
        outputSpeed = config.readDoubleProperty("rollsRUs.motor.output.speed");

    }

    public Command runIntake() {
        return this.runEnd(() -> intakeMotor.set(intakeSpeed), this::stop);
    }

    public Command runOutput() {
        return this.runEnd(() -> intakeMotor.set(outputSpeed), this::stop);
    }

    public Command runIntakeTimed(double seconds) {
        return this.runEnd(() -> intakeMotor.set(intakeSpeed), this::stop).withTimeout(seconds);
    }

    public Command runOutputTimed(double seconds) {
        return this.runEnd(() -> intakeMotor.set(outputSpeed), this::stop).withTimeout(seconds);
    }

    private void stop() {
        intakeMotor.set(0);
    }
}
