package ros.android.msgs;

import org.ros.internal.message.Message;

import app_manager.App;

public abstract interface StatusResponse
  extends Message
{
  public static final String _TYPE = "ros.android.msgs/StatusResponse";
  public static final String _DEFINITION = "# Namespace under which applications will run if connected\nstring application_namespace\n\n# Who is controlling the app manager (i.e. who invited it to send it's\n# control handles). If the empty string, it is not being controlled \n# and subsequently is available\nstring remote_controller\n\n# Current app status, stopped or running\nstring application_status\n\n# Current app details (if running), a default App() (filled with empty strings and lists) otherwise\nrocon_app_manager_msgs/App application";
  
  public abstract String getApplicationNamespace();
  
  public abstract void setApplicationNamespace(String paramString);
  
  public abstract String getRemoteController();
  
  public abstract void setRemoteController(String paramString);
  
  public abstract String getApplicationStatus();
  
  public abstract void setApplicationStatus(String paramString);
  
  public abstract App getApplication();
  
  public abstract void setApplication(App paramApp);
}


/* Location:              /Users/chuckc/robotics/library/rocon_app_manager_msgs-0.6.9.jar!/rocon_app_manager_msgs/StatusResponse.class
 * Java compiler version: 6 (50.0)
 * JD-Core Version:       0.7.1
 */