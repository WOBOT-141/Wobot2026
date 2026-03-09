// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public interface IBotSelector {
    String getName();

    double getDistToFront();
    
    double getfrontLeftOffset();

    double getbackLeftOffset();

    double getbackRightOffset();

    double getfrontRightOffset();

    double getfrontLeftLocationX();

    double getfrontLeftLocationY();

    double getbackLeftLocationX();

    double getbackLeftLocationY();

    double getfrontRightLocationX();

    double getfrontRightLocationY();

    double getbackRightLocationX();

    double getbackRightLocationY();

    double getCameraOffset();

    double rotationsPerInch_rot();
} 