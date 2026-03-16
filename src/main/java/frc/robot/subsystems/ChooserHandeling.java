package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.ICommandFactory;

import java.util.Arrays;
import java.util.Map;

public class ChooserHandeling extends SubsystemBase {

  private final NetworkTableInstance m_inst;
  private final NetworkTable m_table;

  private final Map<String, ICommandFactory> m_commands;

  private String[] m_commandKeys = null;
  private final String m_key;

  private final StringArrayPublisher m_pubChooser;
  private final StringArraySubscriber m_subChooser;
  private final StringSubscriber m_subSelectedAuton;

  public ChooserHandeling(String key, Map<String, ICommandFactory> commands) {

    m_inst = NetworkTableInstance.getDefault();
    m_table = m_inst.getTable("SmartDashboard");

    m_commands = commands;
    m_key = key;

    m_pubChooser = m_table.getStringArrayTopic(key).publish();

    // Smaller default array (was 2000)
    m_subChooser = m_table.getStringArrayTopic(key).subscribe(new String[0]);

    m_subSelectedAuton =
        m_table.getStringTopic("SelectedAuton").subscribe("");
  }

  public String getSelectedAuton() {
    return m_subSelectedAuton.get();
  }

  public void putSelectedAuton() {

    m_table.putValue(
        "SelectedAuton",
        NetworkTableValue.makeString(getSelectedAuton()));
  }

  public String[] getChooserSubscriber() {
    return m_subChooser.get();
  }

  public void putStringPub(String key, String value) {

    m_table.putValue(key, NetworkTableValue.makeString(value));
  }

  public String getStringSub(String key) {

    NetworkTableValue value = m_table.getValue(key);

    if (value == null) {
      return "";
    }

    return value.getString();
  }

  public void putNumber(String key, double value) {

    m_table.putValue(key, NetworkTableValue.makeDouble(value));
  }

  public double getNumber(String key) {

    NetworkTableValue value = m_table.getValue(key);

    if (value == null) {
      return 0.0;
    }

    return value.getDouble();
  }

  public Command getSelected() {

    String selectedAuton = getSelectedAuton();

    ICommandFactory factory = m_commands.get(selectedAuton);

    if (factory == null) {
      System.out.println("Auton not found: " + selectedAuton);
      return null;
    }

    return factory.CreateCommand();
  }

  public String[] getChooserCommandStrings() {

    if (m_commandKeys == null) {

      String[] array = m_commands.keySet().toArray(new String[0]);

      Arrays.sort(array);

      m_commandKeys = array;
    }

    return m_commandKeys;
  }

  @Override
  public void periodic() {

    putSelectedAuton();

    m_pubChooser.set(getChooserCommandStrings());

    m_table.putValue(
        "AutonChooser",
        NetworkTableValue.makeStringArray(getChooserSubscriber()));
  }
}