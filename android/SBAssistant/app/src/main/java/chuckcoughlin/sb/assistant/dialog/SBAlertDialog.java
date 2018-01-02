package chuckcoughlin.sb.assistant.dialog;


import android.app.Activity;
import android.content.DialogInterface;
import android.support.v7.app.AlertDialog;

/**
 * Wraps the alert dialog so it can be used as a yes/no function
 */
public class SBAlertDialog {
    private int state;
    private AlertDialog dialog;
    private Activity context;

    public SBAlertDialog(Activity context, AlertDialog.Builder builder, String yesButton, String noButton) {
        state = 0;
        this.context = context;
        dialog = builder.setPositiveButton(yesButton, new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface dialog, int which) {
                state = 1;
            }})
                .setNegativeButton(noButton, new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int which) {
                        state = 2;
                    }})
                .create();
    }

    public SBAlertDialog(Activity context, AlertDialog.Builder builder, String okButton) {
        state = 0;
        this.context = context;
        dialog = builder.setNeutralButton(okButton, new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface dialog, int which) {
                state = 1;
            }}).create();
    }


    public void setTitle(String m) {
        dialog.setTitle(m);
    }

    public void setMessage(String m) {
        dialog.setMessage(m);
    }

    public boolean show(String m) {
        setMessage(m);
        return show();
    }

    public boolean show() {
        state = 0;
        context.runOnUiThread(new Runnable() {
            public void run() {
                dialog.show();
            }});
        //Kind of a hack. Do we know a better way?
        while (state == 0) {
            try {
                Thread.sleep(1L);
            }
            catch (Exception e) {
                break;
            }
        }
        dismiss();
        return state == 1;
    }

    public void dismiss() {
        if (dialog != null) {
            dialog.dismiss();
        }
        dialog = null;
    }
}