package ros.android.msgs;

import org.ros.internal.message.Message;

public abstract interface GetPlatformInfoResponse
  extends Message
{
  public static final String _TYPE = "rocon_app_manager_msgs/GetPlatformInfoResponse";
  public static final String _DEFINITION = "PlatformInfo platform_info";
  
  public abstract PlatformInfo getPlatformInfo();
  
  public abstract void setPlatformInfo(PlatformInfo paramPlatformInfo);
}


/* Location:              /Users/chuckc/robotics/library/rocon_app_manager_msgs-0.6.9.jar!/rocon_app_manager_msgs/GetPlatformInfoResponse.class
 * Java compiler version: 6 (50.0)
 * JD-Core Version:       0.7.1
 */