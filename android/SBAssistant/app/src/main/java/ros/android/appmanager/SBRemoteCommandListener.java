/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * (MIT License)
 */
package ros.android.appmanager;

/**
 * The discovery fragment implements this interface in response to
 * selection of new applications in the list view
 */

public interface SBRemoteCommandListener {
    /**
     * There was an error in the command attempt.
     * @param key a user-defined value returned in the callback to be used
     *            to identify which command failed
     * @param command the command that was attempted
     * @param reason error description
     */
    public void handleCommandError(String key,String command,String reason);
    /**
     * There was an error in the command attempt.
     * @param key a user-defined value returned in the callback to be used
     *            to identify the command which was executed
     * @param command the command that was executed
     * @param returnValue what was returned from the command
     */
    public void handleCommandCompletion(String key,String command,String returnValue);
}
