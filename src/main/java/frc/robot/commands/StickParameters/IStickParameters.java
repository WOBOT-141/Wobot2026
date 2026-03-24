// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StickParameters;

import edu.wpi.first.wpilibj.PS4Controller.Axis;

/** Add your docs here. */
public interface IStickParameters {

    public AxisParameters getLeftXAxisParams();
    public AxisParameters getLeftYAxisParams();
    public AxisParameters getRightXAxisParams();
    public AxisParameters getRightYAxisParams();

}