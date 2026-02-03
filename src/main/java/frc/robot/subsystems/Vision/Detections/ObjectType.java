// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision.Detections;

public enum ObjectType {
    UNKNOWN((byte) 0), APRILTAG((byte) 1), ALGAE((byte) 2), CORAL((byte) 3);

    public byte bite;

    private ObjectType(byte byt) {
        bite = byt;
    }

    public static ObjectType ObjectBiter(byte bit) {
        switch (bit) {
            case 1:

                return APRILTAG;

            case 2:

                return ALGAE;

            case 3:

                return CORAL;
            default:
                return UNKNOWN;
        }
    }
}
