package frc.robot.subsystems;

/**
 * Drivetrain physical and dynamic specs required for drivetrain interfaces and
 * controllers.
 * <p>
 * TODO: Consolidate with other config schemes.
 */
public class DriveSpecs {
    /**
     * Creates an instance.
     * 
     * @param kTrackWidthM       Distance between left and right wheel centerss
     *                           (>0).
     * @param kWheelBaseM        Distance between front and back wheel centers
     *                           (>=0).
     * @param kTotalMassK        Total mass (>0).
     * @param kFrontOffsetM      Offset from reference center to front edge, along
     *                           X-axis (>=0).
     * @param kBackOffsetMOffset Offset from reference center to back edge, along
     *                           X-axis (<=0).
     * @param kLeftOffsetM       Offset from reference center to left edge, along
     *                           Y-axis (>=0).
     * @param kRightOffsetM      Offset from reference center to right edge, along
     *                           Y-axis (<=0).
     * @param kMaxLinearVelMps   Max linear velocity (>0).
     * @param kMaxLinearAccMpss  Max linear acceleration (>0).
     * @param kMaxAngularVelRps  Max angular velocity (>0).
     * @param kMaxAngularAccRpss Max angular acceleration (>0).
     */
    public DriveSpecs(double kTrackWidthM, double kWheelBaseM, double kTotalMassK,
            double kFrontOffsetM, double kBackOffsetM, double kLeftOffsetM, double kRightOffsetM,
            double kMaxLinearVelMps, double kMaxLinearAccMpss, double kMaxAngularVelRps,
            double kMaxAngularAccRpss) {

        this.kTrackWidthM = kTrackWidthM;
        this.kWheelBaseM = kWheelBaseM;
        this.kTotalMassK = kTotalMassK;

        this.kFrontOffsetM = kFrontOffsetM;
        this.kBackOffsetM = kBackOffsetM;
        this.kLeftOffsetM = kLeftOffsetM;
        this.kRightOffsetM = kRightOffsetM;

        this.kMaxLinearVelMps = kMaxLinearVelMps;
        this.kMaxLinearAccMpss = kMaxLinearAccMpss;

        this.kMaxAngularVelRps = kMaxAngularVelRps;
        this.kMaxAngularAccRpss = kMaxAngularAccRpss;
    }

    public final double kTrackWidthM;
    public final double kWheelBaseM;
    public final double kTotalMassK;

    public final double kFrontOffsetM;
    public final double kBackOffsetM;
    public final double kLeftOffsetM;
    public final double kRightOffsetM;

    public final double kMaxLinearVelMps;
    public final double kMaxLinearAccMpss;

    public final double kMaxAngularVelRps;
    public final double kMaxAngularAccRpss;

    // class

    public static double getChassisRadius(double kTrackWidthM, double kWheelBaseM) {
        return Math.hypot(kWheelBaseM / 2.0, kTrackWidthM / 2.0);
    }

    public static double getMaxAngularVel(double kMaxLinearVelMps, double kChassisRadiusM) {
        return kMaxLinearVelMps / kChassisRadiusM;
    }

    /**
     * Computes the moment of inertia for a rectangular chassis with evenly
     * distributed mass.
     */
    public static double getMomentOfInertia(double kTotalMassK, double kTotalLengthM,
            double kTotalWidthM) {
        double moi = (1.0 / 12.0) * kTotalMassK * (Math.pow(kTotalWidthM, 2) + Math.pow(kTotalLengthM, 2));
        return moi;
    }

    public static double getMaxAngularAcc(boolean isSwerve, double kTotalMassK, double kTotalLengthM,
            double kTotalWidthM, double kMaxLinearAccMpss) {
        // assume mass is equally distributed
        double moi = getMomentOfInertia(kTotalMassK, kTotalLengthM, kTotalWidthM);

        // compute total torque from all modules
        double radius = Math.hypot(kTotalLengthM / 2.0, kTotalWidthM / 2.0);
        int count = isSwerve ? 4 : 2;
        double torque = count * ((kTotalMassK / 4) * kMaxLinearAccMpss * radius);

        // compute maximum angular acceleration
        return torque / moi;
    }
}
