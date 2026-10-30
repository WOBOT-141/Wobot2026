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
        distToFront = 15.25;

        // frontLeftOffset = 0.444092; // 9
        // backLeftOffset = 0.174316; // 10
        // frontRightOffset = 0.008301; // 11
        // backRightOffset = 0.088867; // 12 

        // frontLeftLocationX = -18.25;
        // frontLeftLocationY = 18.25;
        // backLeftLocationX = -18.25;
        // backLeftLocationY = -18.25;
        // frontRightLocationX = 18.25;
        // frontRightLocationY = 18.25;
        // backRightLocationX = 18.25;
        // backRightLocationY = -18.25;

        ////////////
        
        // frontLeftLocationX = -18.25;
        // frontLeftLocationY = 18.25;
        // frontLeftOffset = 0.444092; // 9 Swap with Back left OLD
        // frontLeftOffset = 0.008301; // 11 frontRightOffset = 0.008301; NEW CHANGE
        
        // frontRightLocationX = 18.25;
        // frontRightLocationY = 18.25;
        // frontRightOffset = 0.008301; // 11 Swap with Front Left OLD
        // frontRightOffset = 0.088867;  backRightOffset = 0.088867; NEW CHANGE

        // backLeftLocationX = -18.25;
        // backLeftLocationY = -18.25;
        // backLeftOffset = 0.174316; // 10 Swap with Back right OLD
        // frontLeftOffset = 0.008301; frontLeftOffset = 0.008301; NEW CHANGE

        // backRightLocationX = 18.25;
        // backRightLocationY = -18.25;
        // backRightOffset = 0.088867; // 12  Swap with Front Right OLD
        // backLeftOffset = 0.174316; backLeftOffset = 0.174316; NEW CHANGE

        ///////////////
        
        frontLeftLocationX = 18.25;
        frontLeftLocationY = 18.25;
        frontLeftOffset = 0.008301;
        
        frontRightLocationX = 18.25;
        frontRightLocationY = -18.25;
        frontRightOffset = 0.088867; 

        backLeftLocationX = -18.25;
        backLeftLocationY = 18.25;
        backLeftOffset = 0.424316;

        backRightLocationX = -18.25;
        backRightLocationY = -18.25;
        backRightOffset = 0.174316;




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
