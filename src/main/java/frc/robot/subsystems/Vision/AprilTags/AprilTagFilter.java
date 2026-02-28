package frc.robot.subsystems.Vision.AprilTags;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.AngleMagVector;
import frc.robot.subsystems.Vision.Detections.DetectionList;
import frc.robot.subsystems.Vision.Detections.IDetection;

public class AprilTagFilter {

    private final AprilTagFieldInfo[] m_tags;

    public AprilTagFilter(AprilTagFieldInfo[] tags) {
        m_tags = tags;
    }

    public AprilTagFieldInfo getTag(int tagId) {
        if (tagId < 0 || tagId >= m_tags.length){
            return m_tags[0];
        }

        return m_tags[tagId];
    }

    public boolean isTagOnElement(int tagId, FieldElement fieldElement) {
        return getTag(tagId).FieldElement == fieldElement;
    }

    public Optional<AprilTagFieldInfo> getTagInfoIfElementAndAlliance(int tagId, FieldElement fieldElement) {
        AprilTagFieldInfo tag = getTag(tagId);

        if (tag.FieldElement != fieldElement)
            return Optional.empty();

        Optional<Alliance> matchAlliance = DriverStation.getAlliance();
        
        if (matchAlliance.isEmpty())
            return Optional.of(tag);
        
        if (tag.Alliance != matchAlliance.get())
            return Optional.empty();

        return Optional.of(tag);
    }

    public Optional<AngleMagVector> getFloorVectorFromTag(DetectionList detectionList, FieldElement element) {
        ArrayList<AngleMagVector> floorVectorList = new ArrayList<AngleMagVector>();
        for (int i = 0; i <= detectionList.getSize(); i++){
            IDetection detection = detectionList.getDetection(i);
            if (0 != getTagInfoIfElementAndAlliance(((int)detection.getId()), element).get().Id){
                AngleMagVector floorVector = getFloorVectorFromTag(detection);
                floorVectorList.add(floorVector);
            }
        }
        if (floorVectorList.size() == 0) {
            return Optional.empty();
        };
        double totalMagnitude_in = 0;
        double totalAngle_deg = 0;
        for (int i = 0; i <= floorVectorList.size(); i++) {
            AngleMagVector floorVector = floorVectorList.get(i);
           totalMagnitude_in += floorVector.Magnitude;
           totalAngle_deg += floorVector.Angle; 
        }
        double averageMagnitude_in = totalMagnitude_in/floorVectorList.size();
        double averageAngle_in = totalAngle_deg/floorVectorList.size();
        return Optional.of(new AngleMagVector(averageAngle_in, averageMagnitude_in));

    }

    public AngleMagVector getFloorVectorFromTag(IDetection tagDetection) {
        //This currently only works on center tags
        AprilTagFieldInfo tagInfo = m_tags[((int)tagDetection.getId())];
        double distanceToTag_in = tagDetection.getDistance();
        double tagToCenter_in = Math.sqrt(Math.pow(tagInfo.XToElement , 2) + Math.pow(tagInfo.YToElement, 2));
        double tagToCenterAngle_deg = 180 - Math.abs(tagDetection.getYaw());
        double hubDistance_in = LawOfCosines(tagToCenter_in, distanceToTag_in, tagToCenterAngle_deg);
        double angleToHub_deg = FindSSSAngle(tagToCenter_in, distanceToTag_in, hubDistance_in);

        // TODO Use the Law of Cosines to calculate the vector from the bot to the center of the specified field element
        // based on the information in the provided AprilTagDetection.
        // Only XtoElement and YtoElement matter. Ignore ZtoElement

        throw new UnsupportedOperationException("Not implemented yet");
    }

    public double FindSSSAngle(double sideA, double sideB, double sideC) {
        double numerator = Math.pow(sideA, 2) + Math.pow(sideB, 2) - Math.pow(sideC, 2);
        double denominator = 2 * sideA * sideB;
        return Math.toDegrees(Math.acos(numerator/denominator));
        
    }

    public double LawOfCosines(double sideA, double sideB, double angleC) {
        double sideC = Math.sqrt((Math.pow(sideB, 2) + Math.pow(sideA, 2)) - (2 * sideA * sideB * Math.toDegrees(Math.cos(angleC))));
        return sideC;
    }
}
