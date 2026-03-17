// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision.Detections;

/** Add your docs here. */
public class DetectionList {
    public static double getDistance;
    private final ObjectType _objectType;
    private final int _capacity;
    private final Detection[] _detections;
    private int _size;

    public DetectionList(ObjectType objectType, int capacity) {
        _objectType = objectType;
        _capacity = capacity;

        _detections = new Detection[_capacity];
        for (int i = 0; i < _capacity; ++i) {
            _detections[i] = new Detection();
        }

        _size = 0;
    }

    public ObjectType getObjectType() {
        return _objectType;
    }

    public int getCapacity() {
        return _capacity;
    }

    public int getSize() {
        return _size;
    }

    public void clear() {
        _size = 0;
    }

    public boolean addDetection
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
        _detections[_size].update(tag, distance, bearing, id, yaw, pitch, roll, cameraNum);
        ++_size;
        return _size < _capacity;
    }

    public IDetection getDetection(int index) {
        if (index >= _size || index < 0)
            return NullDetection.Instance;

        return _detections[index];
    }


    public IDetection getDetectionById(int tagId) {
            for (int i = 0; i < _size; i++){
            IDetection currentDetection = getDetection(i);
            if ((int) currentDetection.getId() == tagId){
                return currentDetection;
            }
        }
        return NullDetection.Instance;

    }
}
