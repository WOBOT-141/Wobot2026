// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StickParameters;

/** Add your docs here. */
public class AxisParameters {
    public final double axisMin;
    public final double axisMid;
    public final double axisMax;
    public final double axisDeadband;

    public AxisParameters(double min, double mid, double max, double deadband) {
        axisMin = min;
        axisMid = mid;
        axisMax = max;
        axisDeadband = deadband;
    }
}
