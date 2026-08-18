package frc.robot.Util;

import frc.robot.Constants;

public class ShooterCalculations {
    public double getLaunchAngle(double targetDistance_in) {
        return getLaunchAngle(targetDistance_in, Constants.ShooterConstants.muzzleToRimHeight_in,
                Constants.ShooterConstants.hubRimToCenter_in, targetDistance_in);
    }


    /**
     * @param targetDistance_in       horizontal distance to target
     * @param deltaHeight_in          difference between height of robot and height of target
     * @param rimToTargetDistance_in  horizontal distance from rim to target plane of penetration
     * @param targetHeightAboveRim_in max height above rim that parabola should cross through
     *
     * @return initial launch Angle (degrees)
     */
    public static double getLaunchAngle(double targetDistance_in, double deltaHeight_in, double rimToTargetDistance_in, double targetHeightAboveRim_in) {
        if (targetDistance_in <= 0) return 0;

        double angleStep1 = (2 * targetDistance_in + rimToTargetDistance_in) * (deltaHeight_in)
                + (targetHeightAboveRim_in / rimToTargetDistance_in) * Math.pow(targetDistance_in, 2);
        double angleStep2 = (2 * targetDistance_in + rimToTargetDistance_in) * (targetDistance_in)
                - Math.pow(targetDistance_in, 2);

        if (angleStep2 == 0) return 0;

        return Math.toDegrees(Math.atan((angleStep1 / angleStep2)));
    }


    public static double getLaunchSpeed(double launchAngle_deg, double targetDistance_in, double rimToTargetDistance_in, double targetHeightAboveRim_in) {
        if (launchAngle_deg <= 0 || targetDistance_in <= 0)
            return 0;

        double step1 = Constants.ShooterConstants.g_inps2
                * ((targetDistance_in - Constants.ShooterConstants.hubRimToCenter_in) + rimToTargetDistance_in / 2);
        double step2 = (targetHeightAboveRim_in / rimToTargetDistance_in)
                * (Math.pow(Math.cos(Math.toRadians(launchAngle_deg)), 2))
                + Math.sin(Math.toRadians(launchAngle_deg)) * Math.cos((Math.toRadians(launchAngle_deg)));

        if (step2 == 0) return 0;

        double step3 = (step1 / step2);

        if (step3 < 0) return 0;
        return Math.sqrt(step3);
    }

    
    public static void main(String[] args) {
        // remove this in final code. this is just for testing the results of the
        // calculations.

        double launchAngle1 = getLaunchAngle(35, 43.96, 23.51, 36);
        System.out.println("launch angle " + launchAngle1 + " degrees");

        double launchspeed1 = getLaunchSpeed(launchAngle1, 222.3059, 23.51, 72);
        System.out.println("inital correct velocity " + launchspeed1 + " inches per second.");

        double launchangle2 = getLaunchAngle(106, 43.96, 23.51, 24);
        System.out.println("launch angle " + launchangle2 + " degrees");

        double launchspeed2 = getLaunchSpeed(launchangle2, 222.3059, 23.51, 72);
        System.out.println("inital correct velocity " + launchspeed2 + " inches per second.");
    }

    public static double calculateHeightAboveRim(double angleOfCenterTarget_deg) {
        double MissingAngle_deg = 180 - angleOfCenterTarget_deg - 90;
        return Constants.ShooterConstants.hubRimToCenter_in * Math.tan(Math.toRadians(MissingAngle_deg));
    }

    public static double calculateHeightAboveRim(double angleOfCenterTarget_deg, double distanceRimToTarget) {
        double MissingAngle_deg = 180 - angleOfCenterTarget_deg - 90;
        return distanceRimToTarget * Math.tan(Math.toRadians(MissingAngle_deg));
    }
}