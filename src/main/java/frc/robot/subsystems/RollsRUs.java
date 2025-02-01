package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.generated.TunerConstants;

public class RollsRUs extends SubsystemBase {
    private TalonFX intakeMotor = new TalonFX(TunerConstants.IntakeMotor);
    private Double intakeSpeed;
    private Double outputSpeed;

    public RollsRUs(Config config) {
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = config.readIntegerProperty("rollsRUs.motor.statorCurrent.limit.amps");
        limitConfigs.StatorCurrentLimitEnable = true;

        limitConfigs.SupplyCurrentLimit = config.readIntegerProperty("rollsRUs.motor.supplyCurrent.limit.amps");
        limitConfigs.SupplyCurrentLimitEnable = true;

        intakeMotor.getConfigurator().apply(limitConfigs);

        intakeSpeed = config.readDoubleProperty("rollsRUs.motor.intake.speed");
        outputSpeed = config.readDoubleProperty("rollsRUs.motor.output.speed");

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
