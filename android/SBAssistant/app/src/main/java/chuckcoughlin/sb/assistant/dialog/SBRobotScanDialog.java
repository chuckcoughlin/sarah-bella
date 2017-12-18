package chuckcoughlin.sb.assistant.dialog;

import android.app.Activity;
import android.app.DialogFragment;
import android.app.ProgressDialog;

/**
 * This class encapsulates a ProgressDialog.
 */

public class SBRobotScanDialog extends SBBasicDialogFragment {
    private static final String CLSS = "SBRobotScanDialog";

    public SBRobotScanDialog() {super();}

    public String getDialogType() { return CLSS; }

    public void show(String title, String text) {
        if (dialog != null) {
            this.dismiss();
        }
        Activity activity = ((DialogFragment)handler).getActivity();
        dialog = ProgressDialog.show(activity, title, text, true, true);
        ((ProgressDialog)dialog).setProgressStyle(ProgressDialog.STYLE_SPINNER);
    }
}