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

    public MainBot() {
        distToFront = Units.inchesToMeters(17.375);// in inches

        frontLeftOffset = 0.45093 ; // .818848 //.819824
        backLeftOffset = -0.343994 ; // .624023 //.623291
        backRightOffset = -0.24512; // .022705 //.025146
        frontRightOffset = 0.4975585; // .958008 //.956543

        frontLeftLocationX = -.232; //TODO double check, may not be precise
        frontLeftLocationY = .3556;
        backLeftLocationX = -.232;
        backLeftLocationY = -.3556;
        frontRightLocationX = .232;
        frontRightLocationY = .3556;
        backRightLocationX = .232;
        backRightLocationY = -.3556;
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
    public double getCameraOffset() {
        return 4.125;
    }
}
