package frc.robot.Util;

import frc.lib.AngleMagVector;
import frc.robot.subsystems.Vision.AprilTags.AprilTagFieldInfo;
import frc.robot.subsystems.Vision.AprilTags.AprilTagFilter;
import frc.robot.subsystems.Vision.AprilTags.FieldElement;
import frc.robot.subsystems.Vision.Detections.Detection;
import frc.robot.subsystems.Vision.Detections.ObjectType;

public class HubVectorTests {

    public static void main(String[] args) {
        FindSSSAngle_Equilateral();
        FindSSSAngle_ZeroLengthSide();
        FindSSSAngle_AngleOf0();
        LawOfCosines_Equilateral();
        getFloorVectorFromTag_StraightOn();
        getFloorVectorFromTag_90DegreeAngle();
    }

    public static void PrintTestResults(Object expected, Object actual){
        System.out.printf("expected: %s  actual: %s\n", expected.toString(), actual.toString());
    }

    // @Test
    public static void FindSSSAngle_Equilateral() {
        double angle_deg = AprilTagFilter.FindSSSAngle(1, 1, 1);
        PrintTestResults(60, angle_deg);

        //assertEquals(angle_deg, 60);
    }    

    // @Test 
    public static void FindSSSAngle_ZeroLengthSide() {
        double angle_deg = AprilTagFilter.FindSSSAngle(2, 1, 0);
        PrintTestResults(0, angle_deg);
        //assertEquals(angle_deg, 0);
    }

    //   @Test 
    public static void FindSSSAngle_AngleOf0() {
        double angle_deg = AprilTagFilter.FindSSSAngle(2, 1, 1);
        PrintTestResults(0, angle_deg);
        //assertEquals(angle_deg, 0);
    }

    //  @Test
    public static void LawOfCosines_Equilateral() {
        double side_in = AprilTagFilter.LawOfCosines(2, 2, 60);
        PrintTestResults(2, side_in);
    }

    public static void getFloorVectorFromTag_StraightOn() {
        int tagId = 0;
        AprilTagFieldInfo[] tags = {new AprilTagFieldInfo(tagId,  , null, FieldElement.HUB, 0, 1, 0)}; //TODO: Add a second tag to each of these test methods
        Detection detection = new Detection();
        detection.update(ObjectType.APRILTAG, 0, 1, tagId, 0, 0, 0);

        AprilTagFilter filter = new AprilTagFilter(tags);
        AngleMagVector vector = filter.getFloorVectorFromTag(detection);
        PrintTestResults(0, vector.Angle);
        PrintTestResults(2, vector.Magnitude);
    }

    public static void getFloorVectorFromTag_90DegreeAngle() {
        int tagId = 0;
        AprilTagFieldInfo[] tags = {new AprilTagFieldInfo(tagId,  , null, FieldElement.HUB, 0, 1, 0)};
        Detection detection = new Detection();
        detection.update(ObjectType.APRILTAG, 0, 1, tagId, 90, 0, 0);
        AprilTagFilter filter = new AprilTagFilter(tags);
        AngleMagVector vector = filter.getFloorVectorFromTag(detection);
        PrintTestResults(45, vector.Angle);
        PrintTestResults(Math.sqrt(2), vector.Magnitude);
    }
}
