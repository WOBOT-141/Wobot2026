// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StickParameters;

/** Add your docs here. */
public interface IStickParameters {
    double get_RotateFactor();

    double get_DriveXFactor();

    double get_DriveYFactor();

    double get_ThrottleFactor();

    double get_RotateOffset();

    double get_DriveXOffset();

    double get_DriveYOffset();

    double get_ThrottleOffset();

    double get_knobOffset();

    double get_deadbandX();

    double get_deadbandY();

    double get_deadbandRot();
}
