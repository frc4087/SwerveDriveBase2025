package frc.robot.subsystems;

/**
 * Drivetrain physical and dynamic specs required for interfaces and
 * controllers.
 * <p>
 * TODO: Consolidate with other config schemes.
 * 
 * @param kTrackWidthM       Distance between left and right wheel centerss
 *                           (>0).
 * @param kWheelBaseM        Distance between front and back wheel centers
 *                           (>=0).
 * @param kTotalMassK        Total mass (>0).
 * @param kFrontOffsetM      Offset from reference center to front bumper edge,
 *                           along X-axis (>=0).
 * @param kBackOffsetM       Offset from reference center to back bumper edge,
 *                           along X-axis (<=0).
 * @param kLeftOffsetM       Offset from reference center to left bumper edge,
 *                           along Y-axis (>=0).
 * @param kRightOffsetM      Offset from reference center to right bumper edge,
 *                           along Y-axis (<=0).
 * @param kMaxLinearVelMps   Max linear velocity (>0).
 * @param kMaxLinearAccMpss  Max linear acceleration (>0).
 * @param kMaxAngularVelRps  Max angular velocity (>0).
 * @param kMaxAngularAccRpss Max angular acceleration (>0).
 */
public record DriveSpecs(double kTrackWidthM, double kWheelBaseM, double kTotalMassK,
        double kFrontOffsetM, double kBackOffsetM, double kLeftOffsetM, double kRightOffsetM,
        double kMaxLinearVelMps, double kMaxLinearAccMpss, double kMaxAngularVelRps,
        double kMaxAngularAccRpss) {

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