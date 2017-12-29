package ros.android.msgs;

import org.ros.internal.message.Message;

public abstract interface Constants
  extends Message
{
  public static final String _TYPE = "rocon_app_manager_msgs/Constants";
  public static final String _DEFINITION = "# App manager is not getting controlled, and is thus, available.\nstring NO_REMOTE_CONNECTION=none\n\n# Used to set the app status field.\nstring APP_STOPPED=stopped\nstring APP_RUNNING=running\n";
  public static final String NO_REMOTE_CONNECTION = "none";
  public static final String APP_STOPPED = "stopped";
  public static final String APP_RUNNING = "running";
}


/* Location:              /Users/chuckc/robotics/library/rocon_app_manager_msgs-0.6.9.jar!/rocon_app_manager_msgs/Constants.class
 * Java compiler version: 6 (50.0)
 * JD-Core Version:       0.7.1
 */