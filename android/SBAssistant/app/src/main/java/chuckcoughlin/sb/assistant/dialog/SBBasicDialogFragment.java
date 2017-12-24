package chuckcoughlin.sb.assistant.dialog;

import android.app.Dialog;
import android.app.DialogFragment;
import android.view.View;
import android.view.Window;

/**
 * This is an abstract base class for a collection of dialogs
 * displayed from Fragments.
 */

public abstract class SBBasicDialogFragment extends DialogFragment {
    protected SBDialogCallbackHandler handler = null;

    protected String errorMessage = "";
    protected String selectedButton = "";
    protected Object payload = null;


    public SBBasicDialogFragment() {
        super();
        setShowsDialog(true); // Render as a dialog
        int style = DialogFragment.STYLE_NORMAL;
        int theme = android.R.style.Theme_Holo_Dialog;
        setStyle(style, theme);
    }

    public void dismiss() {
        Dialog dialog = getDialog();
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

    public String getErrorMessage() { return this.errorMessage; }
    protected void setErrorMessage(String msg) { this.errorMessage=msg; }
    public Object getPayload() { return this.payload; }
    protected void setPayload(Object obj) { this.payload=obj; }
    public String getSelectedButton() { return this.errorMessage; }
    protected void setSelectedButton(String btn) { this.selectedButton=btn; }
    public void setHandler(SBDialogCallbackHandler h) {
        this.handler = h;
    }

    protected void decorateDialog(Dialog dialog) {
        dialog.requestWindowFeature(Window.FEATURE_NO_TITLE);
        View v = dialog.getWindow().getDecorView();
        v.setBackgroundResource(android.R.color.transparent);
    }
}
