package frc.robot.Util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.AngleMagVector;
import frc.robot.subsystems.Vision.AprilTags.AprilTagFieldInfo;
import frc.robot.subsystems.Vision.AprilTags.AprilTagFilter;
import frc.robot.subsystems.Vision.AprilTags.FieldElement;
import frc.robot.subsystems.Vision.Detections.Detection;
import frc.robot.subsystems.Vision.Detections.DetectionList;
import frc.robot.subsystems.Vision.Detections.ObjectType;

public class HubVectorTests {

    public static void main(String[] args) {
        // FindSSSAngle_Equilateral();
        // FindSSSAngle_ZeroLengthSide();
        // FindSSSAngle_AngleOf0();
        // LawOfCosines_Equilateral();
        getFloorVectorFromTag_StraightOn();
        getFloorVectorFromTag_90DegreeAngle();
        getFloorVectorToTaggedElement_StraightOnCenterTag();
    }

    public static void PrintTestResults(Object expected, Object actual) {
        System.out.printf("expected: %s  actual: %s\n", expected.toString(), actual.toString());
    }

    // @Test
    public static void FindSSSAngle_Equilateral() {
        double angle_deg = AprilTagFilter.FindSSSAngle(1, 1, 1);
        PrintTestResults(60, angle_deg);

        // assertEquals(angle_deg, 60);
    }

    // @Test
    public static void FindSSSAngle_ZeroLengthSide() {
        double angle_deg = AprilTagFilter.FindSSSAngle(2, 1, 0);
        PrintTestResults(0, angle_deg);
        // assertEquals(angle_deg, 0);
    }

    // @Test
    public static void FindSSSAngle_AngleOf0() {
        double angle_deg = AprilTagFilter.FindSSSAngle(2, 1, 1);
        PrintTestResults(0, angle_deg);
        // assertEquals(angle_deg, 0);
    }

    // @Test
    public static void LawOfCosines_Equilateral() {
        double side_in = AprilTagFilter.LawOfCosines(2, 2, 60);
        PrintTestResults(2, side_in);
    }

    public static void getFloorVectorFromTag_StraightOn() {
        int tagId = 0;
        AprilTagFieldInfo[] tags = { new AprilTagFieldInfo(tagId, tagId, null, FieldElement.HUB, 0, 1, 0) };
        Detection detection = new Detection();
        detection.update(ObjectType.APRILTAG, 0, 1, tagId, 0, 0, 0, 0);

        AprilTagFilter filter = new AprilTagFilter(tags, () -> Optional.empty());
        AngleMagVector vector = filter.getFloorVectorFromCenterTag(detection, 90);
        PrintTestResults(0, vector.Angle);
        PrintTestResults(2, vector.Magnitude);
    }

    public static void getFloorVectorFromTag_90DegreeAngle() {
        int tagId = 0;
        AprilTagFieldInfo[] tags = { new AprilTagFieldInfo(tagId, tagId, null, FieldElement.HUB, 0, 1, 0) };
        Detection detection = new Detection();
        detection.update(ObjectType.APRILTAG, 0, 1, tagId, 90, 0, 0, 0);
        AprilTagFilter filter = new AprilTagFilter(tags, () -> Optional.empty());
        AngleMagVector vector = filter.getFloorVectorFromCenterTag(detection, 0);
        PrintTestResults(45, vector.Angle);
        PrintTestResults(Math.sqrt(2), vector.Magnitude);
    }


    public static void getFloorVectorToTaggedElement_StraightOnCenterTag() {
        int tagId = 1;
        int pairedId = 2;
        AprilTagFieldInfo[] tags = {
                new AprilTagFieldInfo(0, 0, Alliance.Red, FieldElement.NONE, 0, 0, 0),
                new AprilTagFieldInfo(tagId, pairedId, null, FieldElement.HUB, 0, 1, 0),
                new AprilTagFieldInfo(pairedId, tagId, null, FieldElement.HUB, 1, 1, 0)
        };
        DetectionList detectionList = new DetectionList(ObjectType.APRILTAG, 2);
        detectionList.addDetection(ObjectType.APRILTAG, 0.0, 1.0, tagId, 0.0, 0.0, 0.0, 0);
        detectionList.addDetection(ObjectType.APRILTAG, 45, Math.sqrt(2), pairedId, 45, 0.0, 0.0, 0);

        Detection tagDetection = (Detection) detectionList.getDetectionById(tagId);
        Detection pairedDetection = (Detection) detectionList.getDetectionById(pairedId);

        AprilTagFilter filter = new AprilTagFilter(tags, () -> Optional.empty());
        Optional<AngleMagVector> floorVectorOptional = filter.getFloorVectorToTaggedElement(detectionList,
                FieldElement.HUB);

        AngleMagVector floorVector = floorVectorOptional.get();

        if (tagDetection != null && pairedDetection != null) {
            PrintTestResults(0, floorVector.Angle);
            PrintTestResults(2, floorVector.Magnitude);
        }

    }
}
