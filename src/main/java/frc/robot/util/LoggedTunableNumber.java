// Based off of 6328's LoggedTunableNumber class.

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;

public class LoggedTunableNumber {
    
    private DoubleSubscriber subscriber;

    private final String key;
    
    private boolean initialized = false;
    
    // Only used for hasChanged().
    private double lastValue = 0.0;
    
    private double value = 0.0;
    
    public LoggedTunableNumber(String key) {
        this.key = key;
    }

    public LoggedTunableNumber(String key, double defaultValue) {
        this(key);
        initDefault(defaultValue);
    }

    public void initDefault(double defaultValue) {
        if(!initialized) {
            this.value = defaultValue;
            lastValue = value;
            initialized = true;

            if(Constants.tuningMode) {
                DoubleTopic topic = NetworkTableInstance.getDefault().getTable("/Tuning").getDoubleTopic(key);
                topic.publish();
                subscriber = topic.subscribe(defaultValue);
            }
        }
    }
    
    @SuppressWarnings("unused")
    public double get() {
        if(Constants.tuningMode && initialized) {
            value = subscriber.get();
        }
        return value;
    }

    @SuppressWarnings("unused")
    public boolean hasChanged() {
        if(Constants.tuningMode && initialized) {
            value = subscriber.get();
            if(value != lastValue) {
                lastValue = value;
                return true;
            }
        }
        return false;
    }
}