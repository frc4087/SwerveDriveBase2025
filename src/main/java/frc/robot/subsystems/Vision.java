package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PhotonCamera camera;
    public static Vision instance;
    private double yawVal=0;
    private double pitchVal=0;
    private double skewVal=0;
    private double areaVal=0;
    private boolean hasTarget = false;
    private boolean LED_Enable=false;
    // Get the current best target.
// PhotonTrackedTarget target = result.getBestTarget();
// List<PhotonTrackedTarget> targets = result.getTargets();
     // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(28);
    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(105);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(29.7);//29.5
    

    public Vision() {

        //TODO: replace constants stuff
         // Change this to match the name of your camera
        PhotonCamera camera = new PhotonCamera("photonvision");
       // this.camera = new PhotonCamera(Constants.targetCamera);
       // this.camera.setPipelineIndex(Constants.Tape01);
    }

    public static Vision getInstance() {
        if (instance == null) {
          instance = new Vision();
        }
        return instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        var result = this.camera.getLatestResult();
        if (result.hasTargets()) {
            this.yawVal = result.getBestTarget().getYaw();
            this.pitchVal = result.getBestTarget().getPitch();
            this.skewVal = result.getBestTarget().getSkew();
            this.areaVal = result.getBestTarget().getArea();
            this.hasTarget =true;
       
            SmartDashboard.putNumber("Yaw Value", yawVal);
            SmartDashboard.putNumber("Pitch Value", pitchVal);
            SmartDashboard.putNumber("Area Value", areaVal);
            SmartDashboard.putBoolean("LEDs OnOff", this.LED_Enable);
        }
        else{
            this.hasTarget = false;
        }
        if (LED_Enable){
            cameraLEDOn();
            // Set driver mode to off.
            camera.setDriverMode(false);
        }
        else {
            cameraLEDOff();
            // Set driver mode to on.
            camera.setDriverMode(true);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }


    public double getYawVal(){
        return this.yawVal;
    }

    public double getPitchVal(){
        return this.pitchVal;
    }

    public double getSkewVal(){
        return this.skewVal;
    }

    public double getAreaVal(){
        return this.areaVal;
    }

    public boolean hasTargets(){
        return this.hasTarget;
    }

    public void cameraLEDOff() {
        this.camera.setLED(VisionLEDMode.kOff);
    }

    public void cameraLEDOn() {
        //Note that this will just turn the LEDs on
        this.camera.setLED(VisionLEDMode.kOn);
    }

    public void cameraLEDBlink() {
        this.camera.setLED(VisionLEDMode.kBlink);
    }

    public void cameraLEDToggle(){
        if (LED_Enable) {
            LED_Enable = false;
        }
        else {
            LED_Enable = true;
        }
    }

    public void cameraLEDToggleOff(){
        LED_Enable = false;
    }
    public void cameraLEDToggleOn(){ 
        LED_Enable = true;
    }


    public void cameraLED() {
        //Note that this will use the value of the pipeline for the intensity
        this.camera.setLED(VisionLEDMode.kDefault);
    }

    public double getRange(){
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(getPitchVal()));
            double rangeInInches = Units.metersToInches(range);
        
        SmartDashboard.putNumber("Camera Distance", rangeInInches);

        return range;
    }
}
