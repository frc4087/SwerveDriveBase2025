package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private PhotonTrackedTarget target;
  private PhotonTrackedTarget prevTarget;
  private boolean hasTarget;
  //create camera object
  //public static PhotonCamera camera = new PhotonCamera("USB_Camera");
  public static PhotonCamera camera = new PhotonCamera("USB_Camera");
  //create AprilTagFieldLayout object to map tags to locations
  private static AprilTagFieldLayout aprilTagFieldLayout;
  //map camera to the drive train
  public static Transform3d cameraToDrivetrain;
  //create new PhotonVision Pose Estimator
  public static PhotonPoseEstimator m_poseEstimator;
  //lasat frame
  public static PhotonTrackedTarget lastTarget;

  public double timeStamp;
  public double previousTimeStamp;
  public double previousPipelineTimestamp;
  public PhotonPipelineResult previousResult;
  public PhotonPipelineResult result;
  public PhotonTrackedTarget hasTargetCheck;
  public double targetYaw;

  /** Creates a new vision. */
  public Vision() {
    result = camera.getLatestResult();
    if(result.hasTargets() == false){}
    else{
      target = result.getBestTarget();
    }
    prevTarget = target;
    //Transform3d tagTogoal = new Transform3d(new Translation3d(1.5,0.0,0.0), new Rotation3d(0.0,0.0,Math.PI));
    previousPipelineTimestamp = 0.0;
    targetYaw = 0;
    
    //populate transform3d camera realtive to robot
    cameraToDrivetrain = new Transform3d(
      new Translation3d(0.16,0.58,0.0), 
      new Rotation3d(0,0,0)
      );
  }
  public double returnTargetYaw(){
    result = camera.getLatestResult();
    target = result.getBestTarget();
    if (target == null) {

      target = prevTarget;
      

    }
    else {
      prevTarget = target;
    }
    Transform3d transformToTarget = target.getBestCameraToTarget(); 
    targetYaw = transformToTarget.getRotation().getAngle();
    return targetYaw;
  }

  public boolean hasTargets(){
    result = camera.getLatestResult();
    if(!result.hasTargets()){
      return false;
    } else{
      return true;
    }
  }
  public Pose2d getCameraToTarget(){
    result = camera.getLatestResult();
    target = result.getBestTarget();
    //target = result.getBestTarget();
    
    if (target == null) {
      target = prevTarget;
    }
    else {
      prevTarget = target;
    }
    //TODO: BACK UP USING WHILE UNTIL TARGET BACK INTO FRAME
    //TODO: QUERY FOR POSE AT TIMESTAMP TARGET WAS LOST

    //create transfromToTarget using values from camera
    if(target == null){
        return new Pose2d();
    } else {
      Transform3d transformToTarget = target.getBestCameraToTarget();   
      // if(transformToTarget == null){
      //   return null;
      // }
      //translate 3D pose to 2D pose and returns
      Pose2d resultPose = new Pose2d(
        transformToTarget.getX()-0.25, 
        transformToTarget.getY(), 
        transformToTarget.getRotation().toRotation2d()
      );
      return resultPose;
    }
    

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
    
}
