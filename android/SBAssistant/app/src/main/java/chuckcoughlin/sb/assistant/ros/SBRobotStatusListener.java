package chuckcoughlin.sb.assistant.ros;

import chuckcoughlin.sb.assistant.dialog.SBBasicDialogFragment;
import ros.android.util.RobotDescription;

/**
 * The launching fragment for a SBAlertDialog must implement
 * this interface. At the time of launch the launcher must
 * call DialogFragment.setTargetFragment.
 */

public interface SBRobotStatusListener {
    /**
     * Called by the ros helper when the current robot changes status
     * @param robot the current robot
     */
    public void handleStatusChange(RobotDescription robot);
}
