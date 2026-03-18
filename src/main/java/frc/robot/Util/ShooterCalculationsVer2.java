package frc.robot.Util;

import frc.robot.Constants;

public class ShooterCalculationsVer2 {

    public static double getLaunchSpeed(
            double angle_deg,
            double distance_in,
            double shooterHeight_in,
            double targetHeight_in
    ) {
        double theta = Math.toRadians(angle_deg);
        double g = Constants.ShooterConstants.g_inps2;

        double heightDiff = targetHeight_in - shooterHeight_in;

        double numerator = g * Math.pow(distance_in, 2);

        double denominator = 2 * Math.pow(Math.cos(theta), 2)
                * (distance_in * Math.tan(theta) - heightDiff);

        // Prevent invalid math
        if (denominator <= 0) {
            return Double.NaN;
        }

        return Math.sqrt(numerator / denominator);
    }

    public static ShotSolution solveShot(double distance_in) {

        double shooterHeight = Constants.ShooterConstants.muzzleHeight_in;
        double targetHeight = Constants.ShooterConstants.hubRimHeight_in;

        double bestVelocity = Double.POSITIVE_INFINITY;
        double bestAngle = Constants.ShooterConstants.hoodLowerLimit_deg;

        for (double angle = Constants.ShooterConstants.hoodLowerLimit_deg;
             angle <= Constants.ShooterConstants.hoodUpperLimit_deg;
             angle += 0.25) {

            double velocity = getLaunchSpeed(
                    angle,
                    distance_in,
                    shooterHeight,
                    targetHeight
            );

            if (!Double.isNaN(velocity) && velocity < bestVelocity) {
                bestVelocity = velocity;
                bestAngle = angle;
            }
        }

        // Never return NaN
        if (Double.isInfinite(bestVelocity) || Double.isNaN(bestVelocity)) {
            bestVelocity = 0.0;
            bestAngle = Constants.ShooterConstants.hoodLowerLimit_deg;
        }

        double adjustedAngle =
                bestAngle + Constants.ShooterConstants.angleOffset_deg;

        double adjustedVelocity =
                bestVelocity * Constants.ShooterConstants.velocityScale;

        // Clamp angle
        adjustedAngle = Math.max(adjustedAngle, Constants.ShooterConstants.hoodLowerLimit_deg);
        adjustedAngle = Math.min(adjustedAngle, Constants.ShooterConstants.hoodUpperLimit_deg);

        // Final safety
        if (Double.isNaN(adjustedVelocity) || Double.isInfinite(adjustedVelocity)) {
            adjustedVelocity = 0.0;
        }

        return new ShotSolution(adjustedAngle, adjustedVelocity);
    }
}