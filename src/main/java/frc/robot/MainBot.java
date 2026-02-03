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
        distToFront = Units.inchesToMeters(15.25);

        frontLeftOffset  = 0.445312; 
        backLeftOffset   = 0.176270; 
        backRightOffset  = 0.584473; 
        frontRightOffset = 0.507324; 

        frontLeftLocationX = -.229;
        frontLeftLocationY = .229;
        backLeftLocationX = -.229;
        backLeftLocationY = -.229;
        frontRightLocationX = .229;
        frontRightLocationY = .229;
        backRightLocationX = .229;
        backRightLocationY = -.229 ;
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
