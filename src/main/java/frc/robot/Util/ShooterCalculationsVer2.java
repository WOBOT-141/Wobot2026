package frc.robot.Util;

import frc.robot.Constants;

public class ShooterCalculationsVer2 {

    public static class ShotSolution {
        public double angle_deg;
        public double velocity_ips;

        public ShotSolution(double angle_deg, double velocity_ips) {
            this.angle_deg = angle_deg;
            this.velocity_ips = velocity_ips;
        }
    }

    public static double getLaunchSpeed(double angle_deg, double distance_in, double shooterHeight_in, double targetHeight_in) {
        double theta = Math.toRadians(angle_deg);
        double g = Constants.ShooterConstants.g_inps2;

        double heightDiff = targetHeight_in - shooterHeight_in;
        double numerator = g * Math.pow(distance_in, 2);
        double denominator = 2 * Math.pow(Math.cos(theta), 2) * (distance_in * Math.tan(theta) - heightDiff);

        // Prevent impossible shots
        if (denominator <= 0) {
            return Double.NaN;
        }

        return Math.sqrt(numerator / denominator);
    }

    public static ShotSolution solveShot(double distance_in) {

        double shooterHeight = Constants.ShooterConstants.muzzleHeight_in;
        double targetHeight = Constants.ShooterConstants.hubRimHeight_in;

        double bestVelocity = Double.POSITIVE_INFINITY;
        double bestAngle = Double.NaN;

        for (double angle = Constants.ShooterConstants.hoodLowerLimit_deg; angle <= Constants.ShooterConstants.hoodUpperLimit_deg; angle += 0.25) {

            double velocity = getLaunchSpeed(angle, distance_in, shooterHeight, targetHeight);

            if (!Double.isNaN(velocity) && velocity < bestVelocity) {
                bestVelocity = velocity;
                bestAngle = angle;
            }
        }

        // No valid shot
        if (Double.isNaN(bestAngle)) {
            return new ShotSolution(Double.NaN, Double.NaN);
        }

        double adjustedAngle = bestAngle + Constants.ShooterConstants.angleOffset_deg;
        double adjustedVelocity = bestVelocity * Constants.ShooterConstants.velocityScale;

        // Clamp angle to physical limits
        adjustedAngle = Math.max(adjustedAngle, Constants.ShooterConstants.hoodLowerLimit_deg);
        adjustedAngle = Math.min(adjustedAngle, Constants.ShooterConstants.hoodUpperLimit_deg);

        // Prevent invalid velocity
        if (adjustedVelocity < 0 || Double.isNaN(adjustedVelocity) || Double.isInfinite(adjustedVelocity)) {
            return new ShotSolution(Double.NaN, Double.NaN);
        }

        return new ShotSolution(adjustedAngle, adjustedVelocity);
    }

    public static void calculateAndPrint(double distance_in) {

        ShotSolution solution = solveShot(distance_in);

        if (Double.isNaN(solution.angle_deg)) {
            System.out.println("No valid shot for distance: " + distance_in);
            return;
        }

        System.out.println("Distance: " + distance_in + " in");
        System.out.println("Angle: " + solution.angle_deg + " deg");
        System.out.println("Velocity: " + solution.velocity_ips + " in/s");
    }

    public static void main(String[] args) {

        double distance = 220.0;

        calculateAndPrint(distance);
    }
}