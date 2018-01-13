package ros.android.appmanager;

import chuckcoughlin.sb.assistant.dialog.SBBasicDialogFragment;
import ros.android.util.RobotDescription;

/**
 * The discovery fragment implements this interface in response to
 * MasterChecker activities.
 */

public interface SBRobotConnectionHandler {
    /**
     * There was an error in the connection attempt.
     * @param reason error description
     */
    public void handleConnectionError(String reason);
    /**
     * There was an error in the connection attempt.
     * @param reason error description
     */
    public void handleNetworkError(String reason);
    /**
     * There was an error in the connection attempt.
     * @param SSID wifi network name
     * @param wifi wifi user
     * @return true if the re-connection was successful
     */
    public boolean handleWifiReconnection(String SSID,String wifi);
    /**
     * The connection request succeeded. We found the robot.
     * @param appName the application currently running on the robot
     */
    public void receiveApplication(String appName);
    /**
     * The connection request succeeded. We found the robot.
     * @param robot the current robot
     */
    public void receiveConnection(RobotDescription robot);
    /**
     * The wifi connection request succeeded.
     */
    public void receiveWifiConnection();
}
