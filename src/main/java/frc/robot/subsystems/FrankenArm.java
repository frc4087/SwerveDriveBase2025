
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.video.FarnebackOpticalFlow;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Config;
import frc.robot.generated.TunerConstants;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class FrankenArm extends SubsystemBase {
  public TalonFX armMotor = new TalonFX(TunerConstants.ArmMotor);
  private final Integer fwdSpeed;
  private final Integer backwardSpeed;

  public FrankenArm(Config config) {
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = config.readIntegerProperty("rollsRUs.motor.current.limit.amps");
        limitConfigs.StatorCurrentLimitEnable = true;
        armMotor.getConfigurator().apply(limitConfigs);

        fwdSpeed = config.readIntegerProperty("frankenarm.motor.forwards.speed");
        backwardSpeed = config.readIntegerProperty("frankenarm.motor.backwards.speed");
  }

  public Command runFoward() {
    return this.runEnd(() -> armMotor.set(fwdSpeed), this::stop);
  }

  public Command runBack() {
    return this.runEnd(() -> armMotor.set(backwardSpeed), this::stop);
  }

  private void stop() {
    armMotor.set(0);
  }
}
