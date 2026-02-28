package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.Detections.DetectionList;
import frc.robot.subsystems.Vision.Detections.IDetection;
import frc.robot.subsystems.Vision.Detections.ObjectType;

public class VisionServer extends SubsystemBase {

    private AprilTagFieldLayout m_aprilTagFieldLayout;

    private final boolean DEBUG_LOG = false;

    NetworkTableInstance m_nt_Instance;
    NetworkTable m_nt_table;
    NetworkTableEntry aprilTagEntry;
    StringTopic aprilTagStringTopic;
    StringSubscriber subscriber;

    ByteBuffer m_buffer_from_table;

    public VisionServer() {
        m_buffer_from_table = ByteBuffer.allocate(8192);
        m_nt_Instance = NetworkTableInstance.getDefault();
        m_nt_table = m_nt_Instance.getTable("Vision");
        aprilTagEntry = m_nt_table.getEntry("Apriltag");
        // aprilTagStringTopic = m_nt_Instance.getStringTopic("/Vision/Apriltag");
        // subscriber = aprilTagStringTopic.subscribe("skippyskippyskippy",
        // PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10));
        try {
             m_aprilTagFieldLayout = new AprilTagFieldLayout(
                    Filesystem.getDeployDirectory().toPath().resolve("2026-official.json")
            );
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public DetectionList getDetectionList() {
        DetectionList _detectlist = null; // TODO? maybe shouldn't be null?
        if (aprilTagEntry.exists()) {
            m_buffer_from_table.clear();
            m_buffer_from_table.put(aprilTagEntry.getRaw(null));
            m_buffer_from_table.flip();

            int number_of_tags = m_buffer_from_table.getInt();
            int number_of_fields = m_buffer_from_table.getInt();
            if (DEBUG_LOG) {
                System.out.println("num tags " + number_of_tags + " num fields " + number_of_fields);
            }

            _detectlist = new DetectionList(ObjectType.APRILTAG, number_of_tags);

            for (int i = 0; i < number_of_tags; i++) {
                double distance = m_buffer_from_table.getDouble();
                double angle = m_buffer_from_table.getDouble();
                double id = m_buffer_from_table.getDouble();
                double yaw = m_buffer_from_table.getDouble();
                double cameraNum = m_buffer_from_table.getDouble();
                double pitch = 0.0; // m_buffer_from_table.getDouble();
                _detectlist.addDetection(
                        ObjectType.APRILTAG,
                        angle,
                        distance,
                        id,
                        yaw,
                        pitch,
                        cameraNum);
            }

            for (int i = 0; i < _detectlist.getSize(); i++) {
                if (DEBUG_LOG) {
                    System.out.println(_detectlist.getDetection(i).toString());
                }
            }

        } else {
            if (DEBUG_LOG) {
                System.out.println("No dice!");
            }
        }
        return _detectlist;
    }

    public IDetection getClosestDetection() {
        DetectionList detectionList = getDetectionList();
        IDetection closestDetection = detectionList.getDetection(0);
        for (int i = 0; i < detectionList.getSize(); i++) {
            IDetection currentDetection = detectionList.getDetection(i);
            if (currentDetection.getDistance() < closestDetection.getDistance()) {
                closestDetection = currentDetection;
            }
        }
        return closestDetection;
    }

    public double angleYaw() {
        return m_nt_table.getEntry("yaw").getDouble(0);
    }

    public int getID() {
        return (int) m_nt_table.getEntry("id").getDouble(0);
    }

    public double getDistance() {
        return m_nt_table.getEntry("distance").getDouble(0);
    }

    /**
     * Gets the Pose2d of the closest detected tag. This is the position of the
     * tag on the field, not relative to the robot.
     *
     * @return Pose2d of the closest detected tag. If no tags are detected,
     * returns (0, 0, 0).
     */
    public Pose2d getTagPose() {
        Optional<Pose3d> pose = m_aprilTagFieldLayout.getTagPose(getClosestDetection().getId());
        if (pose.isPresent()) {
            return new Pose2d(pose.get().getX(), pose.get().getY(), m_aprilTagFieldLayout.getTagPose(getClosestDetection().getId()).get().getRotation().toRotation2d());
        } else {
            return new Pose2d();
        }
    }

    public double getTimestamp() {
        return Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
        getDetectionList();
    }
}
