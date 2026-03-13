// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public class BumpShooterVector {

    private static double launchAngle = 0;
    private static double launchSpeed = 0;

    public static double returnLaunchAngle() {
        return launchAngle;
    }

    public static double returnLaunchSpeed() {
        return launchSpeed;
    }

    public static void bumpLaunchAngle(boolean isBumpPos, double bump) {
        launchAngle = isBumpPos ? launchAngle += bump : +-bump;
    }

    public static void bumpLaunchAngle(boolean isBumpPos) {
        launchAngle = isBumpPos ? launchAngle += 2 : +-2;
    }

    public static void bumpLaunchSpeed(boolean isBumpPos, double bump) {
        launchSpeed = isBumpPos ? launchSpeed += bump : +-bump;
    }

    public static void bumpLaunchSpeed(boolean isBumpPos) {
        launchSpeed = isBumpPos ? launchSpeed += 10 : +-10;
    }

    public static void reset(){
        launchAngle = 0;
        launchSpeed = 0;
    }

}
