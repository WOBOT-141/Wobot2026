package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public final class DashboardPublisher {

  private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private static final NetworkTable table = inst.getTable("SmartDashboard");

  /**
   * Publishes a double value to the dashboard.
   * 
   * @param key   The key to publish under
   * @param value The value to publish
   */
  public static void publishDouble(String key, double value) {
    table.putValue(key, NetworkTableValue.makeDouble(value));
  }

  /**
   * Publishes an int value to the dashboard.
   * 
   * @param key   The key to publish under
   * @param value The value to publish
   */
  public static void publishInt(String key, int value) {
    table.putValue(key, NetworkTableValue.makeDouble(value));
  }

  /**
   * Publishes a float value to the dashboard.
   * 
   * @param key   The key to publish under
   * @param value The value to publish
   */
  public static void publishFloat(String key, float value) {
    table.putValue(key, NetworkTableValue.makeDouble(value));
  }

  /**
   * Publishes a boolean value to the dashboard.
   * 
   * @param key   The key to publish under
   * @param value The value to publish
   */
  public static void publishBoolean(String key, boolean value) {
    table.putValue(key, NetworkTableValue.makeBoolean(value));
  }

  /**
   * Publishes a string value to the dashboard.
   * 
   * @param key   The key to publish under
   * @param value The value to publish
   */
  public static void publishString(String key, String value) {
    table.putValue(key, NetworkTableValue.makeString(value));
  }

  /**
   * Publishes a char value to the dashboard.
   * 
   * @param key   The key to publish under
   * @param value The value to publish
   */
  public static void publishChar(String key, char value) {
    table.putValue(key, NetworkTableValue.makeString(String.valueOf(value)));
  }

  /**
   * Publishes a double array to the dashboard.
   * 
   * @param key   The key to publish under
   * @param value The value to publish
   */
  public static void publishDoubleArray(String key, double[] value) {
    table.putValue(key, NetworkTableValue.makeDoubleArray(value));
  }

  /**
   * Publishes an int array to the dashboard.
   * 
   * @param key   The key to publish under
   * @param value The value to publish
   */
  public static void publishIntArray(String key, int[] value) {
    double[] converted = new double[value.length];
    for (int i = 0; i < value.length; i++) {
      converted[i] = value[i];
    }
    table.putValue(key, NetworkTableValue.makeDoubleArray(converted));
  }

  /**
   * Publishes a float array to the dashboard.
   * 
   * @param key   The key to publish under
   * @param value The value to publish
   */
  public static void publishFloatArray(String key, float[] value) {
    double[] converted = new double[value.length];
    for (int i = 0; i < value.length; i++) {
      converted[i] = value[i];
    }
    table.putValue(key, NetworkTableValue.makeDoubleArray(converted));
  }

  /**
   * Publishes a boolean array to the dashboard.
   * 
   * @param key   The key to publish under
   * @param value The value to publish
   */
  public static void publishBooleanArray(String key, boolean[] value) {
    table.putValue(key, NetworkTableValue.makeBooleanArray(value));
  }

  /**
   * Publishes a string array to the dashboard.
   * 
   * @param key   The key to publish under
   * @param value The value to publish
   */
  public static void publishStringArray(String key, String[] value) {
    table.putValue(key, NetworkTableValue.makeStringArray(value));
  }

  /**
   * Publishes a char array to the dashboard.
   * 
   * @param key   The key to publish under
   * @param value The value to publish
   */
  public static void publishCharArray(String key, char[] value) {
    String[] converted = new String[value.length];
    for (int i = 0; i < value.length; i++) {
      converted[i] = String.valueOf(value[i]);
    }
    table.putValue(key, NetworkTableValue.makeStringArray(converted));
  }

  /**
   * Gets a double value from the dashboard.
   * 
   * @param key          The key to retrieve
   * @param defaultValue The value to return if the key does not exist
   * @return The double value from the dashboard, or the default value
   */
  public static double getDouble(String key, double defaultValue) {
    var v = table.getValue(key);
    return v != null ? v.getDouble() : defaultValue;
  }

  /**
   * Gets an int value from the dashboard.
   * 
   * @param key          The key to retrieve
   * @param defaultValue The value to return if the key does not exist
   * @return The int value from the dashboard, or the default value
   */
  public static int getInt(String key, int defaultValue) {
    var v = table.getValue(key);
    return v != null ? (int) v.getDouble() : defaultValue;
  }

  /**
   * Gets a float value from the dashboard.
   * 
   * @param key          The key to retrieve
   * @param defaultValue The value to return if the key does not exist
   * @return The float value from the dashboard, or the default value
   */
  public static float getFloat(String key, float defaultValue) {
    var v = table.getValue(key);
    return v != null ? (float) v.getDouble() : defaultValue;
  }

  /**
   * Gets a boolean value from the dashboard.
   * 
   * @param key          The key to retrieve
   * @param defaultValue The value to return if the key does not exist
   * @return The boolean value from the dashboard, or the default value
   */
  public static boolean getBoolean(String key, boolean defaultValue) {
    var v = table.getValue(key);
    return v != null ? v.getBoolean() : defaultValue;
  }

  /**
   * Gets a string value from the dashboard.
   * 
   * @param key          The key to retrieve
   * @param defaultValue The value to return if the key does not exist
   * @return The string value from the dashboard, or the default value
   */
  public static String getString(String key, String defaultValue) {
    var v = table.getValue(key);
    return v != null ? v.getString() : defaultValue;
  }

  /**
   * Gets a char value from the dashboard.
   * 
   * @param key          The key to retrieve
   * @param defaultValue The value to return if the key does not exist
   * @return The char value from the dashboard, or the default value
   */
  public static char getChar(String key, char defaultValue) {
    var v = table.getValue(key);
    if (v == null)
      return defaultValue;
    String s = v.getString();
    return s.isEmpty() ? defaultValue : s.charAt(0);
  }

  /**
   * Gets a double array from the dashboard.
   * 
   * @param key          The key to retrieve
   * @param defaultValue The value to return if the key does not exist
   * @return The double array from the dashboard, or the default value
   */
  public static double[] getDoubleArray(String key, double[] defaultValue) {
    var v = table.getValue(key);
    return v != null ? v.getDoubleArray() : defaultValue;
  }

  /**
   * Gets an int array from the dashboard.
   * 
   * @param key          The key to retrieve
   * @param defaultValue The value to return if the key does not exist
   * @return The int array from the dashboard, or the default value
   */
  public static int[] getIntArray(String key, int[] defaultValue) {
    var v = table.getValue(key);
    if (v == null)
      return defaultValue;

    double[] raw = v.getDoubleArray();
    int[] converted = new int[raw.length];
    for (int i = 0; i < raw.length; i++) {
      converted[i] = (int) raw[i];
    }
    return converted;
  }

  /**
   * Gets a float array from the dashboard.
   * 
   * @param key          The key to retrieve
   * @param defaultValue The value to return if the key does not exist
   * @return The float array from the dashboard, or the default value
   */
  public static float[] getFloatArray(String key, float[] defaultValue) {
    var v = table.getValue(key);
    if (v == null)
      return defaultValue;

    double[] raw = v.getDoubleArray();
    float[] converted = new float[raw.length];
    for (int i = 0; i < raw.length; i++) {
      converted[i] = (float) raw[i];
    }
    return converted;
  }

  /**
   * Gets a boolean array from the dashboard.
   * 
   * @param key          The key to retrieve
   * @param defaultValue The value to return if the key does not exist
   * @return The boolean array from the dashboard, or the default value
   */
  public static boolean[] getBooleanArray(String key, boolean[] defaultValue) {
    var v = table.getValue(key);
    return v != null ? v.getBooleanArray() : defaultValue;
  }

  /**
   * Gets a string array from the dashboard.
   * 
   * @param key          The key to retrieve
   * @param defaultValue The value to return if the key does not exist
   * @return The string array from the dashboard, or the default value
   */
  public static String[] getStringArray(String key, String[] defaultValue) {
    var v = table.getValue(key);
    return v != null ? v.getStringArray() : defaultValue;
  }

  /**
   * Gets a char array from the dashboard.
   * 
   * @param key          The key to retrieve
   * @param defaultValue The value to return if the key does not exist
   * @return The char array from the dashboard, or the default value
   */
  public static char[] getCharArray(String key, char[] defaultValue) {
    var v = table.getValue(key);
    if (v == null)
      return defaultValue;

    String[] raw = v.getStringArray();
    char[] converted = new char[raw.length];
    for (int i = 0; i < raw.length; i++) {
      converted[i] = raw[i].isEmpty() ? '\0' : raw[i].charAt(0);
    }
    return converted;
  }
}