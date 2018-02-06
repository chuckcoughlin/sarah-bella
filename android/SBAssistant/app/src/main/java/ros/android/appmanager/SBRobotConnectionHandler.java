/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * (MIT License)
 */
package ros.android.appmanager;

import ros.android.util.RobotDescription;

/**
 * The discovery fragment implements this interface in response to
 * MasterChecker activities.
 */
public interface SBRobotConnectionHandler extends SBRobotConnectionErrorListener {
    /**
     * There was an error in the connection attempt. In general, we try
     * bluetooth first. If that fails, then wifi.
     * @param networkType bluetooth or wifi
     * @param reason error description
     */
    public void handleNetworkError(String networkType,String reason);
    /**
     * The connection request succeeded. We found the robot.
     * @param appName the application currently running on the robot
     */
    public void receiveApplication(String appName);
    /**
     * The initial request succeeded after the network was estblished.
     * We found the robot.
     * @param robot the current robot
     */
    public void receiveRobotConnection(RobotDescription robot);
    /**
     * The bluetooth or wifi connection request succeeded.
     */
    public void receiveNetworkConnection();
}
