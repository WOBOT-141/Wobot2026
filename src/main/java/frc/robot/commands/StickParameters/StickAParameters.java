// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StickParameters;

/** Add your docs here. */
public class StickAParameters implements IStickParameters {
    double RotateFactor;
    double DriveXFactor;
    double DriveYFactor;
    double ThrottleFactor;
    double RotateOffset;
    double DriveXOffset;
    double DriveYOffset;
    double ThrottleOffset;
    double KnobOffset;
    double deadbandX;
    double deadbandY;
    double deadbandRot;

    public StickAParameters() {
        RotateFactor = 1.35;
        DriveXFactor = 1.26;
        DriveYFactor = 1.23;
        ThrottleFactor = 1.55;
        RotateOffset = -0.056375;
        DriveXOffset = 0.0540; //-0.020093
        DriveYOffset = -0.024228;
        ThrottleOffset = 0;
        deadbandRot = 0.05;
        deadbandX = 0.05;
        deadbandY = 0.05;
        KnobOffset = 1;
    }

    @Override
    public double get_RotateFactor() {
        return RotateFactor;
    }

    @Override
    public double get_DriveXFactor() {
        return DriveXFactor;
    }

    @Override
    public double get_DriveYFactor() {
        return DriveYFactor;
    }

    @Override
    public double get_ThrottleFactor() {
        return ThrottleFactor;
    }

    @Override
    public double get_RotateOffset() {
        return RotateOffset;
    }

    @Override
    public double get_DriveXOffset() {
        return DriveXOffset;
    }

    @Override
    public double get_DriveYOffset() {
        return DriveYOffset;
    }

    @Override
    public double get_ThrottleOffset() {
        return ThrottleOffset;
    }

    @Override
    public double get_knobOffset() {
        return KnobOffset;
    }

    @Override
    public double get_deadbandX() {
        return deadbandX;
    }

    @Override
    public double get_deadbandY() {
        return deadbandY;
    }

    @Override
    public double get_deadbandRot() {
        return deadbandRot;
    }
}