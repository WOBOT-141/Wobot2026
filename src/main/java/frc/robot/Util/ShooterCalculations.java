package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class ShooterCalculations {

    public static double calculateAngleShot(double targetDistance_in) {
        double angleStep1 = (2 * ShooterConstants.dmr_m + ShooterConstants.drt_m) * (ShooterConstants.deltaHprm_m)
                + (ShooterConstants.har_m / ShooterConstants.drt_m) * Math.pow(ShooterConstants.dmr_m, 2);
        double angleStep2 = (2 * ShooterConstants.dmr_m + ShooterConstants.drt_m) * (ShooterConstants.dmr_m)
                - Math.pow(ShooterConstants.dmr_m, 2);
        return (Math.atan(angleStep1 / angleStep2));
    }

    public static double calculateInitialVelocity(double shooterAngle_deg, double targetDistance_in) {
        double shooterAngle_rad = Math.toRadians(shooterAngle_deg);

        double numerator = ShooterConstants.g
                * ((Units.inchesToMeters(targetDistance_in) - ShooterConstants.drc_m) + (ShooterConstants.drt_m / 2));

        double denominator = ((ShooterConstants.har_m / ShooterConstants.drt_m)
                * (Math.pow(Math.cos(shooterAngle_rad), 2))
                + (Math.sin(shooterAngle_rad) * (Math.cos(shooterAngle_rad))));

        return Math.sqrt(numerator / denominator);
    }


    public static double calculateHeightAboveRim(double angleOfCenterTarget_deg) {
        double MissingAngle_deg = 180 - angleOfCenterTarget_deg - 90;
        return ShooterConstants.drt_m * Math.tan(MissingAngle_deg);
    }

    public static double calculateHeightAboveRim(double angleOfCenterTarget_deg, double distanceRimToTarget) {
        double MissingAngle_deg = 180 - angleOfCenterTarget_deg - 90;
        return distanceRimToTarget * Math.tan(MissingAngle_deg);

    }

    public class ShooterConstants {
        private static final double g = 9.817;
        private static final double gf = 32.174; // gravity in ft/s^2

        private static double har_m = 1.228344; // PASS har_m = height above the rim
        private static double drt_m = Units.inchesToMeters(23.51); // PASS ShooterConstants.drt_m = distance rim to
                                                                   // penetration of target
        // plane
        // (how far in should the ball

        private static double dmr_m = .3; // ShooterConstants.dmr_m = horiz dist of muzzle to rim
        private static double drc_m = Units.inchesToMeters(23.51); // drc_m = horiz dist rim to target center
        private static double hrm_m = 1.8; // hrm_m = height of rim
        private static double hm_m = .4; // hm_m = height of muzzle
        // theta = hood angle
        private static double deltaHprm_m = hrm_m + har_m - hm_m;
        /*
         * a = targetDistance_in
         * b = ShooterConstants.drc_m
         * c = ShooterConstants.drt_m
         * d = ShooterConstants har_m
         * l = shooterAngle_deg
         */

    }



}

// MAX ANGLE OF 89 deg, MIN OF 44.5 according to mason

/*
 * 
 * 
 * 
 * 
 * numerator = g((muzzleToTargetCenter_m-drc_m) + (ShooterConstants.drt_m/2))
 * denominator =
 * (har_m/ShooterConstants.drt_m)(cos(theta))^2(sin(theta))(cos(theta))
 * 
 */

/*
 * For angle shot calculation-
 * numerator = (2dmr +
 * ShooterConstants.drt_m)(deltaHprm_m)+(har_m/ShooterConstants.drt_m)
 * ShooterConstants.dmr_m^2
 * denominator = (2dmr + ShooterConstants.drt_m)(ShooterConstants.dmr_m) -
 * (ShooterConstants.dmr_m)^2
 */
