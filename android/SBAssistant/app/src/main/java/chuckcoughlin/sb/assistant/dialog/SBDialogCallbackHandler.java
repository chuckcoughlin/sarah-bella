package chuckcoughlin.sb.assistant.dialog;

import android.app.Fragment;

import java.util.Map;

/**
 * The launching fragment for a SBAlertDialog must implement
 * this interface. At the time of launch the launcher must
 * call DialogFragment.setTargetFragment.
 */

public interface SBDialogCallbackHandler  {
    /**
     * Called by the dialog before it is dismissed.
     * @param dialogId a unique name for the dialog class
     * @param results map containing results
     */
    public void handleDialogResult(String dialogId, Map<String,Object> results);
}
