// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision.Detections;

/** Add your docs here. */
public class NullDetection implements IDetection {
    public static final NullDetection Instance = new NullDetection();

    private NullDetection() {}

    @Override
    public ObjectType getTag() {
        return ObjectType.UNKNOWN;
    }

    @Override
    public double getDistance() {
        return 0;
    }

    @Override
    public double getBearing() {
        return 0;
    }

    @Override
    public int getId() {
        return 0;
    }

    @Override
    public double getYaw() {
        return 0;
    }

    @Override
    public double getPitch() {
        return 0;
    }

    @Override
    public double getRoll() {
        return 0;
    }

    @Override
    public double getCameraNum() {
        return 0;
    }

    @Override
    public boolean isValid() {
        return false;
    }
}
