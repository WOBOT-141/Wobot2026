// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StickParameters;

/** Add your docs here. */
public interface IStickParameters {
    public double getLeftXMax();
    public double getLeftXMid();
    public double getLeftXMin();

    public double getLeftYMax();
    public double getLeftYMid();
    public double getLeftYMin();

    public double getRightXMax();
    public double getRightXMid();
    public double getRightXMin();
    
    public double getRightYMax();
    public double getRightYMid();
    public double getRightYMin();
}
