// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class SummerBot implements IBotSelector {
    double frontLeftOffset;
    double backLeftOffset;
    double backRightOffset;
    double frontRightOffset;
    double distToFront;

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

    public SummerBot() {
        distToFront = Units.inchesToMeters(15.25);

        frontLeftOffset = 0.444092; // 9
        backLeftOffset = 0.174316; // 10
        frontRightOffset = 0.008301; // 11
        backRightOffset = 0.088867; // 12 

        frontLeftLocationX = -.229;
        frontLeftLocationY = .229;
        backLeftLocationX = -.229;
        backLeftLocationY = -.229;
        frontRightLocationX = .229;
        frontRightLocationY = .229;
        backRightLocationX = .229;
        backRightLocationY = -.229;

        driveGearRatio = 6.12;
        wheelCircumference_in = 3.88 * Math.PI;
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
        return "SummerBot";
    }

    @Override
    public double rotationsPerInch_rot() {
        return driveGearRatio / wheelCircumference_in;
    }

    @Override
    public double getCameraOffset() {
        return 12;
    }
}
