package ros.android.msgs;

import java.util.List;
import org.ros.internal.message.Message;

import app_manager.KeyValue;

public abstract interface PairingClient extends Message {
  public static final String _TYPE = "ros.android.msgs/PairingClient";
  public static final String _DEFINITION = "# like \"android\" or \"web\" or \"linux\"\nstring client_type\n\n# like \"intent = ros.android.teleop\" and \"accelerometer = true\", used to choose which ClientApp to use\nKeyValue[] manager_data\n\n# parameters which just get passed through to the client app.\nKeyValue[] app_data";
  
  public abstract String getClientType();
  
  public abstract void setClientType(String paramString);
  
  public abstract List<KeyValue> getManagerData();
  
  public abstract void setManagerData(List<KeyValue> paramList);
  
  public abstract List<KeyValue> getAppData();
  
  public abstract void setAppData(List<KeyValue> paramList);
}


/* Location:              /Users/chuckc/robotics/library/rocon_app_manager_msgs-0.6.9.jar!/rocon_app_manager_msgs/PairingClient.class
 * Java compiler version: 6 (50.0)
 * JD-Core Version:       0.7.1
 */