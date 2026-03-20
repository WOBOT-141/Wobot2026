// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision.Detections;

public class Detection implements IDetection {
    private ObjectType _tag;
    private double _distance;
    private double _bearing;
    private int _id;
    private double _yaw;
    private double _pitch;
    private double _roll;
    private double _cameraNum;
    private boolean _isValid;

    public Detection clone() {
        Detection detect = new Detection();
        detect._distance = _distance;
        detect._bearing = _bearing;
        detect._id = _id;
        detect._yaw = _yaw;
        detect._pitch = _pitch;
        detect._roll = _roll;
        detect._cameraNum = _cameraNum;
        return detect;
    }

    @Override
    public ObjectType getTag() {
        return _tag;
    }

    @Override
    public double getDistance() {
        return _distance;
    }

    @Override
    public double getBearing() {
        return _bearing;
    }

    @Override
    public int getId() {
        return _id;
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
    public double getRoll() {
        return _roll;
    }

    @Override
    public double getCameraNum() {
        return _cameraNum;
    }

    @Override
    public boolean isValid() {
        return _isValid;
    }

    public void update
    (
        ObjectType tag,
        double distance,
        double bearing,
        int id,
        double yaw,
        double pitch,
        double roll,
        double cameraNum
    ) {
        _tag = tag;
        _distance = distance;
        _bearing = bearing;
        _id = id;
        _yaw = yaw;
        _pitch = pitch;
        _roll = roll;
        _cameraNum = cameraNum;
        _isValid = true;
    }

    @Override
    public String toString() {
            return
                _cameraNum+" "+
                _distance+" "+
                _bearing+" "+
                _id+" "+
                _yaw+" "+
                _pitch+" "+
                _roll+" "+
                _tag+" "+
                _isValid
            ;
    }
}
