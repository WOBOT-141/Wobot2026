package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ICommandFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

public class ChooserHandeling extends SubsystemBase {
  private NetworkTableInstance inst;
  private NetworkTable table;

  private String[] optionNames;
  private String key;
  
  private StringArrayPublisher pubChooser;
  private StringArraySubscriber subChooser;
  //private StringPublisher pubString;
  private StringSubscriber subString;

  private final Map<String, ICommandFactory> _commands;
  private String[] _commandKeys = null;

  public ChooserHandeling(String key, Map<String, ICommandFactory> commands) {
    inst = NetworkTableInstance.getDefault();
    _commands = commands;
    table = inst.getTable("SmartDashboard");

    //pubString = table.getStringTopic("SelectedAuton").publish();
    subString = table.getStringTopic("SelectedAuton").subscribe("");
    pubChooser = table.getStringArrayTopic(key).publish();
    subChooser = table.getStringArrayTopic(key).subscribe(new String[2000]);

    this.key = key;

  }
  // public void setSelectedAutonManual(String name){
  //   pubString.set(name);
  // }
  public String getSelectedAuton(){
   String selected =  subString.get();

   return selected;
  }
  // public void setSelectedAuton(){
  //   pubString.set(table.getValue("SelectedAuton").getString());
  // }
  
  public void putSelectedAuton(){
    table.putValue("SelectedAuton", NetworkTableValue.makeString(getSelectedAuton()));
  }

  public String[] listToArray(ArrayList<String> list) {
    String[] arr = new String[list.size()];
    for (int i = 0; i < list.size(); i++) {
      arr[i] = list.get(i);
    }
    return arr;
  }

  public String[] getChooserSubscriber() {
    return subChooser.get();
  }

  public void putStringPub(String key, String value) {
    table.putValue(key, NetworkTableValue.makeString(value));
  }

  public String getStringSub(String key) {
    return table.getValue(key).getString();
  }

  public void putNumber(String key, double value) {
    table.putValue(key, NetworkTableValue.makeDouble(value));
  }

  public double getNumber(String key) {
    return table.getValue(key).getDouble();
  }

  public void readChooser() {
    try {
      optionNames = getChooserCommandStrings();
    } catch (Exception e) {
      e.printStackTrace();
    }
    table.putValue(key,
        NetworkTableValue.makeStringArray(optionNames));
  }

  /**
   * only can pass in 20 values for commands into the chooser
   * we can increase if needed but shouldnt ever need to get that high
   * 
   * @param key
   * @return
   */
  public SendableChooser<Command> newChooser(String key) {
    SendableChooser<Command> chooser = new SendableChooser<>();

    return chooser;
  }

  public void addOption(SendableChooser<Command> chooser, String key, Command value) {
    chooser.addOption(key, value);
  }

  public Command getSelected() {
    String selectedAuton = getSelectedAuton();
    return _commands.get(selectedAuton).CreateCommand();
  }

  public String[] getChooserCommandStrings() {
    if (_commandKeys == null) {
      String[] array = _commands.keySet().toArray(new String[0]);
      Arrays.sort(array);
      _commandKeys = array;
    }

    return _commandKeys;
  }

  @Override
  public void periodic() {

    putSelectedAuton();
    try {
      pubChooser.set(getChooserCommandStrings());
    } catch (Exception e) {
      e.printStackTrace();
    }

    table.putValue("AutonChooser", NetworkTableValue.makeStringArray(getChooserSubscriber()));

  }
}