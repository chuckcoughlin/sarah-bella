package chuckcoughlin.sb.assistant.dialog;

import android.app.Activity;
import android.app.ProgressDialog;

/**
 * This class encapsulates a ProgressDialog.
 */

public class SBProgressDialog {

    private ProgressDialog progress;
    private Activity activity;

    public SBProgressDialog(Activity activity) {
        this.activity = activity;
        progress = null;
    }

    public void dismiss() {
        if (progress != null) {
            progress.dismiss();
        }
        progress = null;
    }

    public void show(String title, String text) {
        if (progress != null) {
            this.dismiss();
        }
        progress = ProgressDialog.show(activity, title, text, true, true);
        progress.setProgressStyle(ProgressDialog.STYLE_SPINNER);
    }
}