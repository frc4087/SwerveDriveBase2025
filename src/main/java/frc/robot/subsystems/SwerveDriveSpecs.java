package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

/**
 * Swerve drive physical and dynamic specs. Intended for use in path following
 * controllers.
 * <p>
 * TODO: Consolidate with Config as appropriate.
 */
public class SwerveDriveSpecs {
    /**
     * For extension only.
     */
    protected SwerveDriveSpecs() {
    }

    public final double kTrackWidthM = Units.inchesToMeters(22.75);
    public final double kWheelBaseM = Units.inchesToMeters(22.75);
    public final double kTotalMassK = Units.lbsToKilograms(150);

    public final double kFrontOffset = -(kWheelBaseM/2.0 + 0.1); // offset to align front edge
    public final double kBackOffset = +(kWheelBaseM/2.0 + 0.1); // offset to align back edge

    public final double kMaxLinearVelMps = 4.5; // Max speed in m/s
    public final double kMaxLinearAccMpss = 2.0; // Max acceleration in m/s²

    public final double kMaxAngularVelRps = getMaxAngularVel();
    public final double kMaxAngularAccRpss = getMaxAngularAcc();

    // class

    public static SwerveDriveSpecs getInstance() {
        if (_instance == null) {
            _instance = new SwerveDriveSpecs();
        }
        return _instance;
    }

    // class personal

    private static SwerveDriveSpecs _instance = null;

    private double getChassisRadius() {
        return Math.hypot(kWheelBaseM / 2.0, kTrackWidthM / 2.0);
    }

    private double getMaxAngularVel() {
        return kMaxLinearVelMps / getChassisRadius();
    }

    private double getMaxAngularAcc() {
        // assume mass is equally distributed
        double momentOfInertia = (1.0 / 12.0) * kTotalMassK
                * (Math.pow(kTrackWidthM, 2) + Math.pow(kWheelBaseM, 2));

        // compute total torque from all 4 modules
        double torque = 4 * ((kTotalMassK / 4) * kMaxLinearAccMpss * getChassisRadius());

        // compute maximum angular acceleration
        return torque / momentOfInertia;
    }
}
