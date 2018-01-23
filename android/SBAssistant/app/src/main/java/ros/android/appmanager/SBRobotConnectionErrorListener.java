package ros.android.appmanager;

import ros.android.util.RobotDescription;

/**
 * The discovery fragment implements this interface in response to
 * application startup issues.
 */

public interface SBRobotConnectionErrorListener {
    /**
     * There was an error in the connection attempt.
     * @param reason error description
     */
    public void handleConnectionError(String reason);
}
