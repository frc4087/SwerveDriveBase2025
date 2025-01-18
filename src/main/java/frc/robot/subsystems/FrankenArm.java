
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;

import au.grapplerobotics.LaserCan;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class FrankenArm extends SubsystemBase {
  
  public LaserCan LaunchSensor = new LaserCan(TunerConstants.LaunchSensor);
  public LaserCan IntakeSensor = new LaserCan(TunerConstants.IntakeSensor);
  public TalonFX IntakeFeedMotor = new TalonFX(TunerConstants.IntakeFeed);
  public TalonFX IntakeCenterMotor = new TalonFX(TunerConstants.IntakeCenter);
  public TalonFX LauncherFeedMotor = new TalonFX(TunerConstants.LaunchFeed);
  public TalonFX LaunchRtFlywheel = new TalonFX(TunerConstants.LaunchRtFlywheel);
  public TalonFX LaunchLtFlywheel = new TalonFX(TunerConstants.LaunchLtFlywheel);
  public final TalonFX armMotor = new TalonFX(TunerConstants.ArmAngle);
  public final CANcoder armSensor = new CANcoder(TunerConstants.ArmSensor);
  public final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
  public DigitalInput limit = new DigitalInput(1);
  Trigger breakBeamTrigger = new Trigger(limit::get);
  //private boolean beamTripped = false;


  public static final double TOLERANCE = 5.0;
  public static final double SETPOINTIntake = 4.0;
  public static final double SETPOINTFar = 2.9;
  public static final double SETPOINTNear = 42;  
  public static final double SETPOINTAmp = 46.467;
  public static final double SETPOINTClose = 12;

  private boolean beamTripped = false;
  //private boolean noteDetected = false;
  public boolean intakeOn = false;

  public FrankenArm() {
    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
    
    // Create MotionMagicConfigs object
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 15000;
    motionMagicConfigs.MotionMagicAcceleration = 6000;

    // Apply MotionMagicConfigs to TalonFXConfiguration
    armMotorConfig.MotionMagic = motionMagicConfigs;

    // Create Slot0Configs object for PID configurations
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.3;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0001;

    // Apply Slot0Configs to TalonFXConfiguration
    armMotorConfig.Slot0 = slot0Configs;

    StatusCode status = armMotor.getConfigurator().apply(armMotorConfig);

    CANcoderConfiguration armSensorConfig = new CANcoderConfiguration();
    status = armSensor.getConfigurator().apply(armSensorConfig);

    //CoastMode();

    // Set initial positions or states
    armSensor.setPosition(0.0); // Set the position of the CANcoder sensor

    // Set neutral mode
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  
    initializeArm();
  }

  private void initializeArm() {
    // Initial calibration
    setArmPosition(SETPOINTIntake); // Move to a known position
}

public void setArmPosition(double position) {
    armMotor.setControl(motionMagic.withPosition(position));
  }
  
public void runLauncher() {
  LaunchRtFlywheel.set(0.8);
  LaunchLtFlywheel.set(0.8);
  LauncherFeedMotor.set(1);
  IntakeFeedMotor.set(1);
  IntakeCenterMotor.set(0);
}

public void runIntake() {
  intakeOn = true;
  LaunchRtFlywheel.set(0.4);
  LaunchLtFlywheel.set(0.4);
  IntakeFeedMotor.set(0.5);
  IntakeCenterMotor.set(0.5);
  LauncherFeedMotor.set(0.5);
  checkBreakBeam();
}

public void runAmp() {
  LaunchRtFlywheel.set(-1.0);
  LaunchLtFlywheel.set(-1.0);
  LauncherFeedMotor.set(-1.0);
}

public void CoastMode(){
IntakeFeedMotor.setNeutralMode(NeutralModeValue.Brake);
IntakeCenterMotor.setNeutralMode(NeutralModeValue.Brake);
LauncherFeedMotor.setNeutralMode(NeutralModeValue.Brake);
}

private void stopMotors() {
  IntakeFeedMotor.set(0);
  IntakeCenterMotor.set(0);
  LauncherFeedMotor.set(0);
}

// public void checkBreakBeam() {
//   if (beamTripped) {
//       // Beam has already been tripped, keep motors off
//       stopMotors();
//       return;
//   }

//   if (limit.get()) {
//       System.out.println("Break beam intact. Motors can run.");
//   } else {
//       System.out.println("Break beam tripped! Stopping motors.");
//       stopMotors();
//       beamTripped = true; // Set the flag to indicate the beam has been tripped
//   }
// }

@Override
    public void periodic() {
        // This method will be called once per scheduler run
        checkBreakBeam();
    }

    private void checkBreakBeam() {
        // If the beam has already been tripped, we don't need to check anymore
        if (beamTripped==true) {
            //stopMotors();
        }

        // If a note hasn't been detected yet, check for it
        // if (!noteDetected) {
        //     if (!limit.get()) {
        //         System.out.println("Note detected! Starting to check break beam.");
        //         noteDetected = true;
        //     }
        //     return; // Exit the method if we're still waiting for a note
        // }

        if(intakeOn==true) {
          BreakBeam();
        }else{
          System.out.println("Intake isn't currently running");
        }

        // At this point, a note has been detected, but the beam hasn't been tripped yet
        // if (limit.get()) {
        //     System.out.println("Break beam intact. Motor can run.");
        //     beamTripped = false;
        // } else {
        //     System.out.println("Break beam tripped! Stopping motor.");
        //     stopMotors();
        //     beamTripped = true; // Set the flag to indicate the beam has been tripped
        // }

        // if (beamTripped==true) {
        //     stopMotors();
        // }
    }

      public void BreakBeam(){
        if (limit.get()) {
            System.out.println("Break beam intact. Motor can run.");
            beamTripped = false;
        } else {
            intakeOn = false;
            System.out.println("Break beam tripped! Stopping motor.");
            //stopMotors();
            beamTripped = true; // Set the flag to indicate the beam has been tripped
        }
      }

    // public void BreakBeam(){
    //   if (limit.get()) {
    //         System.out.println("Break beam intact. Motor can run.");
    //         beamTripped = false;
    //     } else {
    //         System.out.println("Break beam tripped! Stopping motor.");
    //         stopMotors();
    //         beamTripped = true; 
    //         intakeOn = false;
    //     }

    //     if (beamTripped==true) {
    //         stopMotors();
    //     }
    // }

    // public void resetBreakBeamSystem() {
    //     beamTripped = false;
    //     System.out.println("Break beam system reset.");
    // }

    

    // Add other methods for controlling the FrankenArm as needed

// public void checkBreakBeam() {
//   if (beamTripped) {
//       // Beam has already been tripped, keep motors off
//       IntakeFeedMotor.set(0);
//       IntakeCenterMotor.set(0);
//       LauncherFeedMotor.set(0);
//       return;
//   }

//   if (limit.get()) {
//       System.out.println("Break beam intact. Motors can run.");
//   } else {
//       System.out.println("Break beam tripped! Stopping motors.");
//       IntakeFeedMotor.set(0);
//       IntakeCenterMotor.set(0);
//       LauncherFeedMotor.set(0);
//       beamTripped = true; // Set the flag to indicate the beam has been tripped
//   }
// }

// public void checkBreakBeam() {
//   if (limit.get()) {
//   System.out.println("Break beam intact. Motors can run.");
//  } else {
//   System.out.println("Break beam tripped! Stopping motors.");
//   IntakeFeedMotor.set(0);
//   IntakeCenterMotor.set(0);
//   LauncherFeedMotor.set(0);
//   }
//   }

}
