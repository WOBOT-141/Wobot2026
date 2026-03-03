package frc.robot.subsystems.Vision.AprilTags;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.AngleMagVector;
import frc.robot.subsystems.Vision.Detections.DetectionList;
import frc.robot.subsystems.Vision.Detections.IDetection;
import frc.robot.subsystems.Vision.Detections.NullDetection;

public class AprilTagFilter {

    private final AprilTagFieldInfo[] m_tags;

    public AprilTagFilter(AprilTagFieldInfo[] tags) {
        m_tags = tags;
    }

    public AprilTagFieldInfo getTag(int tagId) {
        if (tagId < 0 || tagId >= m_tags.length) {
            return m_tags[0];
        }

        return m_tags[tagId];
    }

    public boolean isTagOnElement(int tagId, FieldElement fieldElement) {
        return getTag(tagId).FieldElement == fieldElement;
    }
    
    public boolean isCenterTag(int tagId, DetectionList detectionList) {
        
    for (int i = 0; i < detectionList.getSize(); i++) {
        IDetection detection = detectionList.getDetection(i);
        AprilTagFieldInfo tag = getTagInfoIfElementAndAlliance((int) detection.getId(), FieldElement.HUB).get();
        if (tag != null){

                if (detectionList.getDetectionById(tag.PairedId) != NullDetection.Instance) {
                    
                }
            
        }

    }
        return m_tags[tagId].XToElement == 0;
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
      
        // TODO: Ethan, you will need to check if the detectionList contains both the center tag and side tag for any hub side
        // (note that I added an "isCenterTag" method).
        // If both pairs exist in the detectionList, use SSSAngle to find the angle between the two detections.
        // From there, find the vector to the center of the hub.
        ArrayList<AngleMagVector> floorVectorList = new ArrayList<AngleMagVector>();
        for (int i = 0; i <= detectionList.getSize(); i++) {
            IDetection detection = detectionList.getDetection(i);
            if (0 != getTagInfoIfElementAndAlliance(((int) detection.getId()), element).get().Id) {
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
        double averageMagnitude_in = totalMagnitude_in / floorVectorList.size();
        double averageAngle_in = totalAngle_deg / floorVectorList.size();
        return Optional.of(new AngleMagVector(averageAngle_in, averageMagnitude_in));

    }

    public AngleMagVector getFloorVectorFromTag(IDetection tagDetection) {
        AprilTagFieldInfo tagInfo = m_tags[((int) tagDetection.getId())];
        double distanceToTag_in = tagDetection.getDistance();
        double tagToCenter_in = Math.sqrt(Math.pow(tagInfo.XToElement, 2) + Math.pow(tagInfo.YToElement, 2));
        double tagToCenterAngle_deg = 180 - Math.toDegrees(Math.atan2(tagInfo.YToElement, tagInfo.XToElement)) + (90 - Math.abs(tagDetection.getYaw()));
        double hubDistance_in = LawOfCosines(tagToCenter_in, distanceToTag_in, tagToCenterAngle_deg);
        double angleToHub_deg = FindSSSAngle(hubDistance_in, distanceToTag_in, tagToCenter_in);
        if (tagDetection.getYaw() < 0) {
            angleToHub_deg = -angleToHub_deg;
        }
        return new AngleMagVector(angleToHub_deg, hubDistance_in);
    }

    /*
     * Returns the angle of the vertex opposite side C
     */
    public static double FindSSSAngle(double sideA, double sideB, double sideC) {
        if (sideA == 0 || sideB == 0 || sideC == 0) {
            return 0;
        }
        double numerator = Math.pow(sideA, 2) + Math.pow(sideB, 2) - Math.pow(sideC, 2);
        double denominator = 2 * sideA * sideB;
        return Math.toDegrees(Math.acos(numerator / denominator));

    }

    public static double LawOfCosines(double sideA, double sideB, double angleC) {
        if (sideA == 0 || sideB == 0) {
            return 0;
        }
        if (angleC == 0) {
            return sideA + sideB;
        }
        double step1 = Math.pow(sideB, 2) + Math.pow(sideA, 2);
        double step2 = 2 * sideA * sideB * Math.cos(Math.toRadians(angleC));
        double sideC = Math.sqrt(step1 - step2);
        return sideC;
    }
}
