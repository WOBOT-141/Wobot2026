// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision.Detections;

public interface IDetection {
    public ObjectType getTag();
    public double getAngle();
    public double getDistance();
    public int getId();
    public double getYaw();
    public double getPitch();
    public double getCameraNum();
    public boolean isValid();
}
