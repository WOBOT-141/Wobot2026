package frc.robot.subsystems.Vision;

import java.nio.ByteBuffer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.Detections.DetectionList;
import frc.robot.subsystems.Vision.Detections.IDetection;
import frc.robot.subsystems.Vision.Detections.ObjectType;

public class VisionServer extends SubsystemBase {

  private final boolean DEBUG_LOG = false;

  NetworkTableInstance m_nt_Instance;
  NetworkTable m_nt_table;
  NetworkTableEntry aprilTagEntry;
  NetworkTableEntry[] aprilTagEntries;
  StringTopic aprilTagStringTopic;
  StringSubscriber subscriber;

  ByteBuffer m_buffer_from_table;

  int m_number_of_cameras;
  int m_number_of_Apriltags;

  DetectionList[] m_detections_Apriltag;

  public VisionServer(int ANumberOfCameras, int AnumberOfTags) {
    m_number_of_cameras = ANumberOfCameras;
    m_number_of_Apriltags = AnumberOfTags;

    m_buffer_from_table = ByteBuffer.allocate(8192);
    m_nt_Instance = NetworkTableInstance.getDefault();
    m_nt_table = m_nt_Instance.getTable("Vision");
    aprilTagEntry = m_nt_table.getEntry("Apriltag");

    aprilTagEntries = new NetworkTableEntry[ANumberOfCameras];
    for (int i = 0; i < aprilTagEntries.length; ++i) {
      aprilTagEntries[i] = m_nt_table.getEntry("Cam"+i+"/Apriltag");
    }

    m_detections_Apriltag = new DetectionList[ANumberOfCameras];
    for (int i = 0; i < m_detections_Apriltag.length; ++i) {
      m_detections_Apriltag[i] = new DetectionList(ObjectType.APRILTAG, AnumberOfTags);
    }
  }

  public void updateDetectionLists() {
    for (int cameraIndex = 0; cameraIndex < m_number_of_cameras; ++cameraIndex) {
      if (aprilTagEntries[cameraIndex].exists()) {
        m_buffer_from_table.clear();
        m_buffer_from_table.put(aprilTagEntries[cameraIndex].getRaw(null));
        m_buffer_from_table.flip();

        int number_of_tags = m_buffer_from_table.getInt();
        int number_of_fields = m_buffer_from_table.getInt();
        if (DEBUG_LOG) {
          System.out.println("num tags " + number_of_tags + " num fields " + number_of_fields);
        }

        boolean not_done_with_detections_from_camera = true;
        do {
          double distance = m_buffer_from_table.getDouble();
          double angle = m_buffer_from_table.getDouble();
          int id = (int) m_buffer_from_table.getDouble();
          double yaw = m_buffer_from_table.getDouble();
          double pitch = m_buffer_from_table.getDouble();
          double roll = m_buffer_from_table.getDouble();
          double cameraNum = m_buffer_from_table.getDouble();

          if (id != 0) {
            m_detections_Apriltag[cameraIndex].addDetection(
              ObjectType.APRILTAG,
              distance,
              angle,
              id,
              yaw,
              pitch,
              roll,
              cameraNum
            );
          } else {
            not_done_with_detections_from_camera = false;
          }

        } while (not_done_with_detections_from_camera);

      }
    }
  }

  public DetectionList getApriltagDetectionList(int ACameraSelect) {
    return m_detections_Apriltag[ACameraSelect];
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
        int id = (int) m_buffer_from_table.getDouble();
        double yaw = m_buffer_from_table.getDouble();
        double pitch = m_buffer_from_table.getDouble();
        double roll = m_buffer_from_table.getDouble();
        double cameraNum = m_buffer_from_table.getDouble();
        
        _detectlist.addDetection(
            ObjectType.APRILTAG,
            distance,
            angle,
            id,
            yaw,
            pitch,
            roll,
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

  public IDetection getClosestDetection() {
    DetectionList detectionList = getDetectionList();
    IDetection closestDetection = detectionList.getDetection(0);
    for (int i = 0; i < detectionList.getSize(); i++){
      IDetection currentDetection = detectionList.getDetection(i);
      if (currentDetection.getDistance() < closestDetection.getDistance())
      {
          closestDetection = currentDetection;
      }
    }
    return closestDetection;
  }
  

  public double angleYaw() {
    return m_nt_table.getEntry("yaw").getDouble(0);
  }

  @Override
  public void periodic() {
    //getDetectionList();
    updateDetectionLists();
  }
}


