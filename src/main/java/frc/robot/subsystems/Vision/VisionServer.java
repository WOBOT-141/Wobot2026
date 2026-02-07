package frc.robot.subsystems.Vision;

import java.nio.ByteBuffer;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.Detections.Detection;
import frc.robot.subsystems.Vision.Detections.DetectionList;
import frc.robot.subsystems.Vision.Detections.IDetection;
import frc.robot.subsystems.Vision.Detections.ObjectType;

public class VisionServer extends SubsystemBase {

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
  }

  public DetectionList getDetectionList() {
    DetectionList _detectlist = null; // TODO? maybe shouldn't be null?
    if (aprilTagEntry.exists()) {
      m_buffer_from_table.clear();
      m_buffer_from_table.put(aprilTagEntry.getRaw(null));
      m_buffer_from_table.flip();

      int number_of_tags = m_buffer_from_table.getInt();
      int number_of_fields = m_buffer_from_table.getInt();
      if (DEBUG_LOG)
        System.out.println("num tags " + number_of_tags + " num fields " + number_of_fields);

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
        if (DEBUG_LOG)
          System.out.println(_detectlist.getDetection(i).toString());
      }

    } else {
      if (DEBUG_LOG)
        System.out.println("No dice!");
    }
    return _detectlist;
  }

  //public Detection getClosestDetection() {
    //DetectionList detectionList = getDetectionList();
    //for (double detectionDistance = 0; detectionDistance <= ; detectionDistance++);
  

  public double angleYaw() {
    return m_nt_table.getEntry("yaw").getDouble(0);
  }

  @Override
  public void periodic() {
    getDetectionList();
  }
}


