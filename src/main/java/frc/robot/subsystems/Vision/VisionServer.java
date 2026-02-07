// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.Detections.DetectionList;
import frc.robot.subsystems.Vision.Detections.IDetection;
import frc.robot.subsystems.Vision.Detections.NullDetection;
import frc.robot.subsystems.Vision.Detections.ObjectType;

/**
 * Handles incoming TCP connections from the vision computer, and handles
 * the messages which describe what targets are in view.
 * 
 * At the moment, only one connection at a time is accepted. If another
 * connection
 * is received, the existing connection (if any) is closed.
 * 
 * If a connection is closed, all detections from that connection are lost.
 * All detections of a given target type are received in a single message.
 * Subsequent
 * messages of the same target type overwrite the previous detections for that
 * target type. If no targets of a given type are visible, the vision computer
 * will send an "empty" message for that type, meaning that the "Number of
 * Targets"
 * field will be 0, and the message ends after the "Reserved" field.
 * E.g., the following indicates 0 AprilTags are in view: 57 4f 42 4f 54 01 00
 * 00 00 00 00 00 00 00
 * 
 * Messages describe 0 - MAX_TARGETS targets of a specified type (i.e., AprilTag, Fuel, etc).
 * 
 * The main details of message processing, as well as the arrays which store the
 * currently known
 * targets, are in the inner class ServerVisionConnection.
 * 
 * Message Format:
 * +------------------+-----------------------+-----------------------------+--------------------+----------------------------------------+
 * | Header (5 bytes) | Target Type (1 byte) | Number of Targets (4 bytes) |
 * Reserved (4 bytes) | Targets (32 bytes * Number of Targets) |
 * +------------------+-----------------------+-----------------------------+--------------------+----------------------------------------+
 * | WOBOT (ASCII) | See ObjectType enum | int | int | See Target below |
 * +------------------+-----------------------+-----------------------------+--------------------+----------------------------------------+
 * 
 * Target:
 * +--------------------+-----------------+--------------+---------------+
 * | Distance (8 bytes) | Angle (8 bytes) | ID (8 bytes) | Yaw (8 bytes) |
 * +--------------------+-----------------+--------------+---------------+
 * | double | double | double | double |
 * +--------------------+-----------------+--------------+---------------+
 * 
 * NOTE: All numerical data types are stored in network order (i.e., Big
 * Endian).
 * 
 * TODO:
 * - Clean up the ServerVisionConnection class (perhaps using state pattern)
 * - Add better logging
 * - Review exception handling
 * - Review error handling when setting up socket. We get a null ref exception
 * if we cannot bind atm
 * - Add methods to retrieve current detections
 * - Break out some of the subclasses into their own files
 */
public class VisionServer extends SubsystemBase {
  private static final int PORT = 1190;

  private static final String SD_KEY_ACCEPTED = "Accepted?";

  private ServerSocketChannel _serverSocketChannel;
  private Selector _selector;
  private ServerVisionConnection _connection;

  private NetworkTableInstance m_nt_instance;
  private NetworkTable m_nt_table;

  private final IntegerPublisher _runningPublisher;
  private final BooleanPublisher _connectedPublisher;
  private long _loopCount;

  /** Creates a new ServerVision. */
  public VisionServer() {
    // Attempt to open Socket
    try {
      _serverSocketChannel = ServerSocketChannel.open();
      _serverSocketChannel.configureBlocking(false);
      _serverSocketChannel.socket().bind(new InetSocketAddress(PORT));
      _selector = Selector.open();
      _serverSocketChannel.register(_selector, SelectionKey.OP_ACCEPT);
    } catch (IOException e) {
      System.out.println("Failed to open socket");
    }

    m_nt_instance = NetworkTableInstance.getDefault();
    m_nt_table = m_nt_instance.getTable("Vision");

    IntegerTopic runningTopic = m_nt_table.getIntegerTopic("Running");
    _runningPublisher = runningTopic.publish();

    BooleanTopic connectedTopic = m_nt_table.getBooleanTopic("Connected");
    _connectedPublisher = connectedTopic.publish();
    _connectedPublisher.set(false);
    
  }

  public IDetection closestDetect(ObjectType object) {
    if (_connection == null)
      return NullDetection.Instance;

    return _connection.getClosestDetect(object);
  }

  public IDetection getAprilTagById(int id) {
    if (_connection == null)
      return NullDetection.Instance;

    return _connection.getAprilTagById(id);
  }

  public IDetection detectIndex(ObjectType object, int ind) {
    if (_connection == null)
      return NullDetection.Instance;

    return _connection.getDetectIndex(object, ind);
  }

  @Override
  public void periodic() {
    _runningPublisher.set(++_loopCount);

    // This method will be called once per scheduler run
    try {
      _selector.selectNow((SelectionKey key) -> {
        if (key.isAcceptable()) {
          try {
            if (_connection != null) {
              disconnect();
            }
            acceptConnection();
          } catch (IOException e) {
            e.printStackTrace();
            System.out.println("Not accepting");
            SmartDashboard.putBoolean(SD_KEY_ACCEPTED, false);
          }
        }
      });
    } catch (IOException e) {
      System.out.println("Failed to accept");
    }

    if (_connection != null) {
      try {
        _connection.process();
        // ZDW SmartDashboard.putNumberArray("testarray", )
      } catch (Exception ex) {
        System.out.println("process exception: " + ex.getMessage());
      }

      if (_connection.isClosed()) {
        disconnect();
      }
    }
  }

  private void acceptConnection() throws IOException {
    SocketChannel channel = _serverSocketChannel.accept();
    SmartDashboard.putBoolean(SD_KEY_ACCEPTED, true);
    channel.configureBlocking(false);
    _connection = new ServerVisionConnection(channel);
    _connectedPublisher.set(true);
    System.out.println("Accepted Channel");
  }

  private void disconnect() {
    if (!_connection.isClosed())
      _connection.close();

    System.out.println("Connection closed");
    SmartDashboard.putBoolean(SD_KEY_ACCEPTED, false);
    _connection = null;
    _connectedPublisher.set(false);
  }

  private class ServerVisionConnection {
    // WOBOT
    private static final byte[] HEADER = { 0x57, 0x4f, 0x42, 0x4f, 0x54 };

    private static final int MAX_TARGETS = 10;
    private static final int TARGET_SIZE = 40;
    private static final int NUM_TARGET_TYPES = 4;

    private static final String SD_KEY_MESSAGES_PROCESSED = "Messages Processed";
    private static final String SD_KEY_INCOMPLETE_MESSAGES = "Incomplete Messages";

    // Detections are index by ObjectType
    private final DetectionList[] _detections;

    private final SocketChannel _socketChannel;
    private boolean _closed = false;

    private final ByteBuffer _socketBuff;

    private ParseState _parseState = ParseState.READ_HEADER;

    // State for READ_HEADER
    private int _headerIndex = 0;

    // State for READ_TARGET_TYPE
    private ObjectType _targetType;

    // State for READ_TOTAL_TARGETS
    private int _totalTargets = 0;
    private int _totalTargetBytesRead = 0;

    // State for READ_TARGETS
    private final ByteBuffer _targetBuff;
    private int _targetsRead = 0;

    // These are for debugging--can probably remove.
    private int _incompleteMessagesProcessed = 0;
    private int _totalMessagesProcessed = 0;
    private boolean _messageCompleted = true;

    public ServerVisionConnection(SocketChannel socketChannel) {
      _socketChannel = socketChannel;
      _socketBuff = ByteBuffer.allocate(8192);
      _targetBuff = ByteBuffer.allocate(TARGET_SIZE);

      _detections = new DetectionList[NUM_TARGET_TYPES];
      for (int i = 0; i < NUM_TARGET_TYPES; ++i) {
        _detections[i] = new DetectionList(ObjectType.ObjectBiter((byte) i), MAX_TARGETS);
      }

      SmartDashboard.putNumber(SD_KEY_MESSAGES_PROCESSED, _totalMessagesProcessed);
      SmartDashboard.putNumber(SD_KEY_INCOMPLETE_MESSAGES, _incompleteMessagesProcessed);

      SetState(ParseState.READ_HEADER);
    }

    public void process() {
      if (_closed)
        return;

      try {
        _socketBuff.clear();
        int bytesRead = _socketChannel.read(_socketBuff);
        if (bytesRead == -1) {
          _closed = true;
        } else if (bytesRead > 0) {
          try {
            _socketBuff.flip();
            while (_socketBuff.hasRemaining()) {
              byte b = _socketBuff.get();

              switch (_parseState) {
                case READ_HEADER:
                  ProcessNextHeaderByte(b);
                  break;

                case READ_TARGET_TYPE:
                  ProcessNextTargetTypeByte(b);
                  break;

                case READ_TOTAL_TARGETS:
                  ProcessNextTargetCountByte(b);
                  break;

                case READ_TARGETS:
                  ProcessNextTargetByte(b);
                  break;

                default:
                  System.out.println("Invalid parse state: " + _parseState);
                  SetState(ParseState.READ_HEADER);
                  break;
              }
            }

          } catch (IndexOutOfBoundsException e) {
            System.out.println("Out of bounds");
          }
        }

        /*** Print out the closest to the smart dashboard */
        IDetection closest = getClosestDetect(ObjectType.APRILTAG);
        SmartDashboard.putNumber("ClosestAprilID", closest.getId());
        SmartDashboard.putNumber("ClosestAprilYaw", closest.getYaw());
        SmartDashboard.putNumber("ClosestAprilAngle", closest.getAngle());
        SmartDashboard.putNumber("ClosestAprilDistance", closest.getDistance());

      } catch (IOException e) {
        System.out.println("IO Exception");
      }
    }

    private void ProcessNextHeaderByte(byte b) {
      if (b == HEADER[_headerIndex])
        _headerIndex++;
      else
        _headerIndex = 0;

      if (_headerIndex >= HEADER.length)
        SetState(ParseState.READ_TARGET_TYPE);
    }

    private void ProcessNextTargetTypeByte(byte b) {
      _targetType = ObjectType.ObjectBiter(b);
      SetState(ParseState.READ_TOTAL_TARGETS);
    }

    private void ProcessNextTargetCountByte(byte b) {
      // The second 4 bytes are the second dimension which we don't care about
      if (_totalTargetBytesRead < 4)
        _totalTargets = (_totalTargets << 1) | b;

      ++_totalTargetBytesRead;

      if (_totalTargetBytesRead == 8) {
        if (_targetType.bite < NUM_TARGET_TYPES)
          _detections[_targetType.bite].clear();

        if (_totalTargets == 0) {
          LogMessageCompleted();
          SetState(ParseState.READ_HEADER);
        } else {
          SetState(ParseState.READ_TARGETS);
        }
      }
    }

    private void ProcessNextTargetByte(byte b) {
      _targetBuff.put(b);

      if (!_targetBuff.hasRemaining()) {
        _targetBuff.flip();

        int targetType = _targetType.bite;
        if (targetType < NUM_TARGET_TYPES) {
          double angle = _targetBuff.getDouble();
          double distance = _targetBuff.getDouble();
          double id = _targetBuff.getDouble();
          double yaw = _targetBuff.getDouble();
          double pitch = _targetBuff.getDouble();
          double cameraNum = _targetBuff.getDouble();
          _detections[targetType].addDetection(_targetType, distance, angle, id, yaw, pitch, cameraNum);
        }

        ++_targetsRead;
        _targetBuff.clear();
      }

      if (_targetsRead == Math.min(_totalTargets, MAX_TARGETS)) {
        LogMessageCompleted();
        SetState(ParseState.READ_HEADER);
      }
    }

    private void SetState(ParseState state) {
      _parseState = state;

      switch (state) {
        case READ_HEADER:
          _headerIndex = 0;
          if (!_messageCompleted) {
            LogIncompleteMessage();
          }
          break;

        case READ_TARGET_TYPE:
          _messageCompleted = false;
          break;

        case READ_TOTAL_TARGETS:
          _totalTargets = 0;
          _totalTargetBytesRead = 0;
          break;

        case READ_TARGETS:
          _targetBuff.clear();
          _targetsRead = 0;
          break;

        default:
          break;
      }
    }

    private void LogMessageCompleted() {
      _totalMessagesProcessed++;
      SmartDashboard.putNumber(SD_KEY_MESSAGES_PROCESSED, _totalMessagesProcessed);
      _messageCompleted = true;
    }

    private void LogIncompleteMessage() {
      _incompleteMessagesProcessed++;
      SmartDashboard.putNumber(SD_KEY_INCOMPLETE_MESSAGES, _incompleteMessagesProcessed);
      _messageCompleted = true;
    }

    public void close() {
      try {
        _socketChannel.close();
      } catch (IOException e) {
        System.out.println("Error while closing _socketChannel. Message: " + e.getMessage());
      }
    }

    public boolean isClosed() {
      return _closed;
    }

    private enum ParseState {
      READ_HEADER, READ_TARGET_TYPE, READ_TOTAL_TARGETS, READ_TARGETS
    }

    public IDetection getClosestDetect(ObjectType obj) {
      DetectionList detectionList = _detections[obj.bite];
      if (detectionList.getSize() == 0)
        return NullDetection.Instance;

      IDetection closest = detectionList.getDetection(0);
      for (int i = 1; i < detectionList.getSize(); i++) {
        IDetection d = detectionList.getDetection(i);
        if (d.getDistance() <= closest.getDistance()) {
          closest = d;
        }
      }

      return closest;
    }

    public IDetection getAprilTagById(int id) {
      DetectionList detectionList = _detections[ObjectType.APRILTAG.bite];

      for (int i = 0; i < detectionList.getSize(); ++i) {
        IDetection d = detectionList.getDetection(i);
        if (d.getId() == id)
          return d;
      }

      return NullDetection.Instance;
    }

    public IDetection getDetectIndex(ObjectType obj, int index) {
      return _detections[obj.bite].getDetection(index);
    }
  }
}
