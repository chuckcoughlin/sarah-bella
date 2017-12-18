package chuckcoughlin.sb.assistant.dialog;

import android.app.Dialog;
import android.app.DialogFragment;
import android.support.v4.app.Fragment;

/**
 * This is an abstract base class for a collection of dialogs
 * displayed from Fragments.
 */

public abstract class SBBasicDialogFragment extends DialogFragment {
    protected SBDialogCallbackHandler handler = null;
    protected Dialog dialog = null;

    public SBBasicDialogFragment() {
        super();
        setShowsDialog(true); // Render as a dialog
    }

    public void dismiss() {
        if (dialog != null) {
            dialog.dismiss();
        }
        dialog = null;
    }
    /**
     * Each dialog class must have a unique name.
     * @return the name of the dialog class
     */
    public abstract String getDialogType();

    public void setHandler(SBDialogCallbackHandler h) {
        this.handler = h;
    }

}
