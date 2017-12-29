package ros.android.msgs;

import java.util.List;
import org.ros.internal.message.Message;

import app_manager.App;

public abstract interface GetAppListResponse extends Message
{
  public static final String _TYPE = "ros.android.msgs/GetAppListResponse";
  public static final String _DEFINITION = "App[] available_apps\nApp[] running_apps";
  
  public abstract List<App> getAvailableApps();
  
  public abstract void setAvailableApps(List<App> paramList);
  
  public abstract List<App> getRunningApps();
  
  public abstract void setRunningApps(List<App> paramList);
}


/* Location:              /Users/chuckc/robotics/library/rocon_app_manager_msgs-0.6.9.jar!/rocon_app_manager_msgs/GetAppListResponse.class
 * Java compiler version: 6 (50.0)
 * JD-Core Version:       0.7.1
 */