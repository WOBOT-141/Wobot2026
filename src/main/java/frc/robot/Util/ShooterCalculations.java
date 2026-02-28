package frc.robot.Util;

import edu.wpi.first.math.util.Units;

public class ShooterCalculations {
    public double getLaunchAngle(double targetDistance_in){
return getLaunchAngle(targetDistance_in, ShooterConstants.deltaHprm_in);
    }

    public static double getLaunchAngle(double targetDistance_in, double deltaHeight_in) {
        double angleStep1 = (2 * ShooterConstants.dmr_in + ShooterConstants.drt_in) * (deltaHeight_in)
                + (ShooterConstants.targetHeight_in / ShooterConstants.drt_in) * Math.pow(ShooterConstants.dmr_in,2);
        double angleStep2 = (2 * ShooterConstants.dmr_in + ShooterConstants.drt_in) * (ShooterConstants.dmr_in)
                - Math.pow(ShooterConstants.dmr_in, 2);
        return (Math.atan(Math.toRadians(angleStep1 / angleStep2)));
    }
    
    public static double getLaunchSpeed(double launchAngle_deg,double targetDistance_in) {
        double step1 = ShooterConstants.g
                * ((targetDistance_in - ShooterConstants.drc_in) + ShooterConstants.drt_in / 2);
        double step2 = (ShooterConstants.targetHeight_in / ShooterConstants.drt_in) * (Math.pow(Math.toRadians(Math.cos(launchAngle_deg)), 2))
                + (Math.sin(Math.toRadians(launchAngle_deg)) * Math.toRadians(Math.cos(launchAngle_deg)));
        double step3 = (step1 / step2);
        return Math.sqrt(step3);
    }

    public static void main(String[] args) {

        double angleOfCenterTarget_deg = 45.0; // example angle
        double initialVelocity = getLaunchSpeed(angleOfCenterTarget_deg, 60.0); // example distance
        System.out.println("Initial velocity: " + initialVelocity + " m/s");
    }

    public static double calculateHeightAboveRim(double angleOfCenterTarget_deg) {
        double MissingAngle_deg = 180 - angleOfCenterTarget_deg - 90;
        return ShooterConstants.drt_in * Math.tan(Math.toRadians(MissingAngle_deg));
    }

    public static double calculateHeightAboveRim(double angleOfCenterTarget_deg, double distanceRimToTarget) {
        double MissingAngle_deg = 180 - angleOfCenterTarget_deg - 90;
        return distanceRimToTarget * Math.tan(Math.toRadians(MissingAngle_deg));

    }

    public class ShooterConstants {
        private static final double g = Units.metersToInches(9.807);

        private static double targetHeight_in = 48.36; //MAKE A PARAMETER AT SOME PT  har_m = height above the rim
        private static double drt_in = 23.51; //MAKE A PARAMETER AT SOME PT ShooterConstants.drt_m = distance rim to penetration of target plane
                                              // (how far in should the ball
        
        private static double dmr_in = 11.811; // ShooterConstants.dmr_m = horiz dist of muzzle to rim
        private static double drc_in = 23.51; // drc_m = horiz dist rim to target center
        private static double hrm_in = 70.8661; // hrm_m = height of rim
        private static double hm_in = 24.3359; // hm_m = height of muzzle
        // theta = hood angle
        private static double deltaHprm_in = hrm_in + targetHeight_in - hm_in;

    }



}

// MAX ANGLE OF 89 deg, MIN OF 44.5 according to mason

/*
 * 
 * 
 * 
 * 
 * numerator = g((target-drc_m) + (ShooterConstants.drt_m/2))
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
