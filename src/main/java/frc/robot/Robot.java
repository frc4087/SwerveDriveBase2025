// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {

  private final CommandXboxController driverController = new CommandXboxController(0);
  //private final CommandXboxController operatorController = new CommandXboxController(1);

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  PhotonCamera camera = new PhotonCamera("photonvision");


  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);


  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.auto().runInit();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
    
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.auto().runPeriodic();
  }

  @Override
  public void autonomousExit() {
    m_robotContainer.auto().runExit();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // Calculate drivetrain commands from Joystick values
        double forward = -driverController.getLeftY() * Config.Constants.Swerve.kMaxLinearSpeed;
        double strafe = -driverController.getLeftX() * Config.Constants.Swerve.kMaxLinearSpeed;
        double turn = -driverController.getRightX() * Config.Constants.Swerve.kMaxAngularSpeed;
        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        double targetRange = 0.0;
                var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetRange =
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        0.5, // Measured with a tape measure, or in CAD.
                                        1.435, // From 2024 game manual for ID 7
                                        Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch()));

                        targetVisible = true;
                    }
                }
            }
          }
        //   }
        //   // Correct pose estimate with vision measurements
        // var visionEst = vision.getEstimatedGlobalPose();
        // visionEst.ifPresent(
        //         est -> {
        //             // Change our trust in the measurement based on the tags we can see
        //             var estStdDevs = vision.getEstimationStdDevs();

        //             drivetrain.addVisionMeasurement(
        //                     est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        //         });
        // Command drivetrain motors based on target speeds
      //CommandSwerveDrivetrain.drive(forward, strafe, turn);
      
  }
  double VISION_DES_ANGLE_deg = 0;
  double VISION_TURN_kP = 0;
  double VISION_TURN_kI = 0;
  double VISION_TURN_kD = 0;
  double VISION_STRAFE_kP = 0;
  double VISION_STRAFE_kI = 0;
  double VISION_STRAFE_kD = 0;
  double VISION_DES_RANGE_m = 0;

  // Auto-align when requested
//    if (driverController.getAButton() && targetVisible) {
//     // Driver wants auto-alignment to tag 7
//     // And, tag 7 is in sight, so we can turn toward it.
//     // Override the driver's turn and fwd/rev command with an automatic one
//     // That turns toward the tag, and gets the range right.
//     turn =
//             (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * Config.Constants.Swerve.kMaxAngularSpeed;
//     forward =
//             (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP * Config.Constants.Swerve.kMaxLinearSpeed;

// }



        // Put debug information to the dashboard
        // SmartDashboard.putBoolean("Vision Target Visible", targetVisible);

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
