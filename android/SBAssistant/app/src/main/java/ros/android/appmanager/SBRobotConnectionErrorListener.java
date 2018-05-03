/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * (MIT License)
 */
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
    public void handleRobotCommunicationError(String reason);
}
