package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class KlimbKardashian extends SubsystemBase {
    private TalonFX climbMotor;

    private final Double upSetpoint;
	private final Double downSetpoint;

    //private final Double resetSetpoint;

    final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

    public KlimbKardashian(Config config) {
        var climbMotorPort = config.readIntegerProperty("ports.climb.motor");
        climbMotor = new TalonFX(climbMotorPort);
        var limitConfigs = new CurrentLimitsConfigs();

        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		talonFXConfigs.Slot0.kS = 0.0;
		talonFXConfigs.Slot0.kG = 0.0;
		talonFXConfigs.Slot0.kV = 0.0;
		talonFXConfigs.Slot0.kP = 1.0;
		talonFXConfigs.Slot0.kI = 0.0;
		talonFXConfigs.Slot0.kD = 0.0;
		
		talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = 160.0;
		talonFXConfigs.MotionMagic.MotionMagicAcceleration = 240.0;
		talonFXConfigs.MotionMagic.MotionMagicJerk = 3200.0;

        limitConfigs.StatorCurrentLimit = config.readIntegerProperty("klimbKardashian.motor.statorCurrent.limit.amps");
        limitConfigs.StatorCurrentLimitEnable = true;

        limitConfigs.SupplyCurrentLimit = config.readIntegerProperty("klimbKardashian.motor.supplyCurrent.limit.amps");
        limitConfigs.SupplyCurrentLimitEnable = true;

        climbMotor.getConfigurator().apply(limitConfigs);
        climbMotor.getConfigurator().apply(talonFXConfigs, 0.050);
        climbMotor.setNeutralMode(NeutralModeValue.Brake);

        upSetpoint = config.readDoubleProperty("klimbKardashian.motor.up.setpoint");
		downSetpoint = config.readDoubleProperty("klimbKardashian.motor.down.setpoint");
        //resetSetpoint = config.readDoubleProperty("klimbKardashian.motor.reset.setpoint");
    }

    public Command climbIn() {
		return this.run(() -> {
			climbMotor.setControl(m_motmag
				.withSlot(0)
				.withPosition(upSetpoint)
			);
		});
	}

	public Command climbOut() {
		return this.run(() -> {
			climbMotor.setControl(m_motmag
				.withSlot(0)
				.withPosition(downSetpoint));
		});
	}

    /*public Command restClimb() {
            return this.run(() -> {
                climbMotor.setControl(m_motmag
                    .withSlot(0)
                    .withPosition(resetSetpoint));
            });
        }

    private void stopReset() {
		climbMotor.set(0.0);
		climbMotor.setPosition(0.0);
	}*/

}
