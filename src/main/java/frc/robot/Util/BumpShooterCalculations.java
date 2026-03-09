// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public class BumpShooterCalculations {

    private static double heightAboveRimBump;
    private static double rimToTargetBump;

    public static double returnBumpedHeight() {
        return heightAboveRimBump;
    }

    public static double returnBumpedRimToTarget() {
        return rimToTargetBump;
    }

    public static void bumpHeight(boolean isBumpPos, double bump) {
        heightAboveRimBump = isBumpPos ? heightAboveRimBump += bump : +-bump;
    }

    public static void bumpHeight(boolean isBumpPos) {
        heightAboveRimBump = isBumpPos ? heightAboveRimBump += 2 : +-2;
    }

    public static void bumpRimToTarget(boolean isBumpPos, double bump) {
        rimToTargetBump = isBumpPos ? rimToTargetBump += bump : +-bump;
    }

    public static void bumpRimToTarget(boolean isBumpPos) {
        rimToTargetBump = isBumpPos ? rimToTargetBump += 2 : +-2;
    }

}
