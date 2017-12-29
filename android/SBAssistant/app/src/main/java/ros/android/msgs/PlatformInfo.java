package ros.android.msgs;

import org.ros.internal.message.Message;

import app_manager.Icon;

public abstract interface PlatformInfo
  extends Message
{
  public static final String _TYPE = "rocon_app_manager_msgs/PlatformInfo";
  public static final String _DEFINITION = "# Provides platform details from the app manager.\n\n######################## Platform Triple ########################\n\nstring PLATFORM_ANY=*\nstring PLATFORM_LINUX=linux\nstring PLATFORM_WINDOZE=windoze\nstring PLATFORM_ANDROID=android\n\nstring SYSTEM_CUSTOM=custom\nstring SYSTEM_ROS=ros\nstring SYSTEM_OPROS=opros\n\n# Valid robot types, though this is totally not\n# official, and we aren't relying on it.\nstring ROBOT_ANY=*\nstring ROBOT_PC=pc\nstring ROBOT_ROBOSEM=robosem\nstring ROBOT_KOBUKI=kobuki\nstring ROBOT_TURTLEBOT=turtlebot\n\n########################### Variables ###########################\n\nstring platform\nstring system\nstring robot\nstring name\nIcon icon";
  public static final String PLATFORM_ANY = "*";
  public static final String PLATFORM_LINUX = "linux";
  public static final String PLATFORM_WINDOZE = "windoze";
  public static final String PLATFORM_ANDROID = "android";
  public static final String SYSTEM_CUSTOM = "custom";
  public static final String SYSTEM_ROS = "ros";
  public static final String SYSTEM_OPROS = "opros";
  public static final String ROBOT_ANY = "*";
  public static final String ROBOT_PC = "pc";
  public static final String ROBOT_ROBOSEM = "robosem";
  public static final String ROBOT_KOBUKI = "kobuki";
  public static final String ROBOT_TURTLEBOT = "turtlebot";
  
  public abstract String getPlatform();
  
  public abstract void setPlatform(String paramString);
  
  public abstract String getSystem();
  
  public abstract void setSystem(String paramString);
  
  public abstract String getRobot();
  
  public abstract void setRobot(String paramString);
  
  public abstract String getName();
  
  public abstract void setName(String paramString);
  
  public abstract Icon getIcon();
  
  public abstract void setIcon(Icon paramIcon);
}


/* Location:              /Users/chuckc/robotics/library/rocon_app_manager_msgs-0.6.9.jar!/rocon_app_manager_msgs/PlatformInfo.class
 * Java compiler version: 6 (50.0)
 * JD-Core Version:       0.7.1
 */