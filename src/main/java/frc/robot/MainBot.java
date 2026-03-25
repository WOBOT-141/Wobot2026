// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class MainBot implements IBotSelector {
    double distToFront;
    double frontLeftOffset;
    double backLeftOffset;
    double backRightOffset;
    double frontRightOffset;

    double frontLeftLocationX;
    double frontLeftLocationY;
    double backLeftLocationX;
    double backLeftLocationY;
    double frontRightLocationX;
    double frontRightLocationY;
    double backRightLocationX;
    double backRightLocationY;

    double driveGearRatio;
    double wheelCircumference_in;

    public MainBot() {
        distToFront = 15.25;

        frontLeftOffset  = 0.551025;
        backLeftOffset   = 0.340332;
        frontRightOffset = 0.004395;
        backRightOffset  = 0.745605;

        frontLeftLocationX = -18.25;
        frontLeftLocationY = 18.25;
        backLeftLocationX = -18.25;
        backLeftLocationY = -18.25;
        frontRightLocationX = 18.25;
        frontRightLocationY = 18.25;
        backRightLocationX = 18.25;
        backRightLocationY = -18.25;

        driveGearRatio = 5.14;
        wheelCircumference_in = 4 * Math.PI;
    }

    @Override
    public double getfrontLeftOffset() {
        return frontLeftOffset;
    }

    @Override
    public double getbackLeftOffset() {
        return backLeftOffset;
    }

    @Override
    public double getbackRightOffset() {
        return backRightOffset;
    }

    @Override
    public double getfrontRightOffset() {
        return frontRightOffset;
    }

    @Override
    public double getfrontLeftLocationX() {
        return frontLeftLocationX;
    }

    @Override
    public double getfrontLeftLocationY() {
        return frontLeftLocationY;
    }

    @Override
    public double getbackLeftLocationX() {
        return backLeftLocationX;
    }

    @Override
    public double getbackLeftLocationY() {
        return backLeftLocationY;
    }

    @Override
    public double getfrontRightLocationX() {
        return frontRightLocationX;
    }

    @Override
    public double getfrontRightLocationY() {
        return frontRightLocationY;
    }

    @Override
    public double getbackRightLocationX() {
        return backRightLocationX;
    }

    @Override
    public double getbackRightLocationY() {
        return backRightLocationY;
    }

    @Override
    public double getDistToFront() {
        return distToFront;
    }

    @Override
    public String getName() {
        return "MainBot";
    }

    @Override
    public double rotationsPerInch_rot() {
        return driveGearRatio / wheelCircumference_in;
    }

    @Override
    public double getCameraOffset() {
        return 4.125;
    }
}
