package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;

public class RollsRUs extends SubsystemBase {
    public TalonFX intakeMotor = new TalonFX(TunerConstants.IntakeMotor);

    public RollsRUs() {
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = 40;
        limitConfigs.StatorCurrentLimitEnable = true;

        intakeMotor.getConfigurator().apply(limitConfigs);
    }

    public void runIntake() {
        intakeMotor.set(1.0);
    }

    public void runOutput() {
        intakeMotor.set(-1.0);
    }
}
