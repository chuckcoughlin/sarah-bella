package ros.android.msgs;

import org.ros.internal.message.Message;

public abstract interface Status
  extends Message
{
  public static final String _TYPE = "rocon_app_manager_msgs/Status";
  public static final String _DEFINITION = "---\n# Namespace under which applications will run if connected\nstring application_namespace\n\n# Who is controlling the app manager (i.e. who invited it to send it's\n# control handles). If the empty string, it is not being controlled \n# and subsequently is available\nstring remote_controller\n\n# Current app status, stopped or running\nstring application_status\n\n# Current app details (if running), a default App() (filled with empty strings and lists) otherwise\nrocon_app_manager_msgs/App application\n";
}


/* Location:              /Users/chuckc/robotics/library/rocon_app_manager_msgs-0.6.9.jar!/rocon_app_manager_msgs/Status.class
 * Java compiler version: 6 (50.0)
 * JD-Core Version:       0.7.1
 */