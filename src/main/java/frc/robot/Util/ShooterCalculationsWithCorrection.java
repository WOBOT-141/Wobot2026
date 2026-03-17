package frc.robot.Util;

import frc.robot.Constants;

public class ShooterCalculationsWithCorrection {

    public double getLaunchAngle(double targetDistance_in) {

        double targetHeightAboveRim = calculateHeightAboveRimFromDistance(targetDistance_in, Constants.ShooterConstants.hubRimToCenter_in);

        return getLaunchAngle(targetDistance_in, Constants.ShooterConstants.muzzleToRimHeight_in, Constants.ShooterConstants.hubRimToCenter_in, targetHeightAboveRim);
    }

    public static double getLaunchAngle(double targetDistance_in, double deltaHeight_in, double rimToTargetDistance_in, double targetHeightAboveRim_in) {

        double angleStep1 = (2 * targetDistance_in + rimToTargetDistance_in) * deltaHeight_in + (targetHeightAboveRim_in / rimToTargetDistance_in) * Math.pow(targetDistance_in, 2);
        double angleStep2 = (2 * targetDistance_in + rimToTargetDistance_in) * targetDistance_in - Math.pow(targetDistance_in, 2);

        if (angleStep2 == 0) {
            return Double.NaN;
        }

        return Math.toDegrees(Math.atan(angleStep1 / angleStep2));
    }

    public static double getLaunchSpeed(double launchAngle_deg, double targetDistance_in, double rimToTargetDistance_in, double targetHeightAboveRim_in) {

        double theta = Math.toRadians(launchAngle_deg);

        double step1 = Constants.ShooterConstants.g_inps2 * ((targetDistance_in - rimToTargetDistance_in) + rimToTargetDistance_in / 2.0);
        double step2 = (targetHeightAboveRim_in / rimToTargetDistance_in) * Math.pow(Math.cos(theta), 2) + Math.sin(theta) * Math.cos(theta);

        // Prevent divide-by-zero or negative sqrt
        if (step2 <= 0) {
            return Double.NaN;
        }

        double result = step1 / step2;

        if (result <= 0) {
            return Double.NaN;
        }

        return Math.sqrt(result);
    }

    public static double calculateHeightAboveRimFromDistance(
            double distance_in,
            double rimToTargetDistance
    ) {
 
        return distance_in * (rimToTargetDistance / (distance_in + rimToTargetDistance));
    }

    public static void main(String[] args) {

        double distance = 222.3059;
        double height = calculateHeightAboveRimFromDistance(distance, 23.51);
        double angle = getLaunchAngle(distance, 43.96, 23.51, height);
        double speed = getLaunchSpeed(angle, distance, 23.51, height);

        System.out.println("Angle: " + angle);
        System.out.println("Velocity (in/s): " + speed);
    }
}