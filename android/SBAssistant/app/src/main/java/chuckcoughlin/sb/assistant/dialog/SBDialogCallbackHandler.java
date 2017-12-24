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
     * @param dialog the instance of dialog that is displayed
     */
    public void handleDialogResult(SBBasicDialogFragment dialog);
}
