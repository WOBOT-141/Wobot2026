package frc.robot.subsystems.Vision.AprilTags;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

    public AprilTagFieldInfo getTagInfoIfElementAndAlliance(int tagId, FieldElement fieldElement) {
        AprilTagFieldInfo tag = getTag(tagId);

        if (tag.FieldElement != fieldElement)
            return m_tags[0];

        Optional<Alliance> matchAlliance = DriverStation.getAlliance();
        
        if (matchAlliance.isEmpty())
            return tag;
        
        if (tag.Alliance != matchAlliance.get())
            return m_tags[0];

        return tag;
    }
}
