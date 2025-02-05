package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.common.networktables.PacketSubscriber;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision {

    
    PhotonCamera camera = new PhotonCamera("myCamera");
    PacketSubscriber<PhotonPipelineResult> resultSubscriber;

public List<PhotonPipelineResult> getAllUnreadResults() {
            List<PhotonPipelineResult> ret = new ArrayList<>();
  // Grab the latest results. We don't care about the timestamps from NT - the metadata header has
        // this, latency compensated by the Time Sync Client
        var changes = resultSubscriber.getAllChanges();
        for (var c : changes) {
            var result = c.value;
           // checkTimeSyncOrWarn(result);
            ret.add(result);
        }

        return ret;

}
}
