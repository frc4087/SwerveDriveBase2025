
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
//import com.ctre.phoenix6.motorcontrol.can.TalonFXConfiguration;
import frc.robot.systems.autonomous.AutonomousController;

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
  public TalonFX armMotor;
  private final Double fwdSetpoint;
  private final Double backwardSetpoint;

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  public FrankenArm(Config config) {
        var armMotorPort = config.readIntegerProperty("ports.arm.motor");
        armMotor = new TalonFX(armMotorPort);
        var limitConfigs = new CurrentLimitsConfigs();

        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonFXConfigs.Slot0.kS = 0.0;
        talonFXConfigs.Slot0.kG = 0.25;
        talonFXConfigs.Slot0.kV = 0.0;
        talonFXConfigs.Slot0.kP = .5;
        talonFXConfigs.Slot0.kI = 0.0;
        talonFXConfigs.Slot0.kD = 0.0;
 
        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = 2.0;
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = 160.0;
        talonFXConfigs.MotionMagic.MotionMagicJerk = 1600.0;

        limitConfigs.StatorCurrentLimit = config.readIntegerProperty("frankenarm.motor.statorCurrent.limit.amps");
        limitConfigs.StatorCurrentLimitEnable = true;

        limitConfigs.SupplyCurrentLimit = config.readIntegerProperty("frankenarm.motor.supplyCurrent.limit.amps");
        limitConfigs.SupplyCurrentLimitEnable = true;

        armMotor.getConfigurator().apply(limitConfigs);
        armMotor.getConfigurator().apply(talonFXConfigs, 0.050);

        fwdSetpoint = config.readDoubleProperty("frankenarm.motor.fwd.Setpoint");
        backwardSetpoint = config.readDoubleProperty("frankenarm.motor.backward.Setpoint");
  }

  public Command goZero() {
    return this.run(() -> {
      armMotor.setControl(new PositionVoltage(0)
      .withSlot(0)
      .withPosition(0.0));
    });
  }


  public Command goNine() {
  return this.run(() -> {
    armMotor.setControl(new PositionVoltage(0)
      .withSlot(0)
      .withPosition(9.0));
  });
  }

  public void periodicConfig() {
    //m_motmag.Slot = 0;
    //armMotor.setControl(m_motmag.withPosition(0.0));
  }
  
  // private void stop() {
  //   armMotor.set(0.0);
  // }
}

