// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision.Detections;

public class Detection implements IDetection {
    private ObjectType _tag;
    private double _angle;
    private double _distance;
    private double _id;
    private double _yaw;
    private double _pitch;
    private double _cameraNum;
    private boolean _isValid;

    public Detection clone() {
        Detection detect = new Detection();
        detect._angle = _angle;
        detect._distance = _distance;
        detect._id = _id;
        detect._yaw = _yaw;
        detect._cameraNum = _cameraNum;
        return detect;
    }

    @Override
    public ObjectType getTag() {
        return _tag;
    }

    @Override
    public double getAngle() {
        return _angle;
    }

    @Override
    public double getDistance() {
        return _distance;
    }

    @Override
    public int getId() {
        return (int) _id;
    }

    @Override
    public double getYaw() {
        return _yaw;
    }

    @Override
    public double getPitch() {
        return _pitch;
    }

    @Override
    public double getCameraNum() {
        return _cameraNum;
    }

    @Override
    public boolean isValid() {
        return _isValid;
    }

    public void update(ObjectType tag, double angle, double distance, double id, double yaw, double pitch,double cameraNum) {
        _tag = tag;
        _angle = angle;
        _distance = distance;
        _id = id;
        _yaw = yaw;
        _pitch = pitch;
        _cameraNum = cameraNum;
        _isValid = true;
    }
}
