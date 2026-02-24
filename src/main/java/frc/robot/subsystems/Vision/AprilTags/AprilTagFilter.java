package frc.robot.subsystems.Vision.AprilTags;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagDetection;
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
        for (int i = 0; i <= detectionList.getSize(); i++){
            IDetection detection = detectionList.getDetection(i);
            if (0 != getTagInfoIfElementAndAlliance(((int)detection.getId()), element).get().Id){
                getFloorVectorFromTag(detection);
                //return Optional.of()
            }
            else {
                return Optional.empty();
            }
        }

        // TODO Iterate through the detectionList, use getTagInfoIfElementAndAlliance to determine if the detection is relevant.
        // If so, call getFloorVectorFromTag method to get the vector based on that detection.
        // Then return the average of all those vectors in an Optional.
        // If there are no relevant detections, return an empty Optional


        throw new UnsupportedOperationException("Not implemented yet");
    }

    public AngleMagVector getFloorVectorFromTag(IDetection tagDetection) {

        AprilTagFieldInfo tagInfo = m_tags[((int)tagDetection.getId())];
        double distanceToTag = tagInfo.XToElement;
        double distanceToCenter_in = 24.5;

        // TODO Use the Law of Cosines to calculate the vector from the bot to the center of the specified field element
        // based on the information in the provided AprilTagDetection.
        // Only XtoElement and YtoElement matter. Ignore ZtoElement

        throw new UnsupportedOperationException("Not implemented yet");
    }
}
