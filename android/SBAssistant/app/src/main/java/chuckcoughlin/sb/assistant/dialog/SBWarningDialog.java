package chuckcoughlin.sb.assistant.dialog;


import android.app.Activity;
import android.app.Dialog;
import android.app.DialogFragment;
import android.content.DialogInterface;
import android.os.Bundle;
import android.support.v7.app.AlertDialog;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBRosManager;

/**
 * Wraps the alert dialog so it can be used to post an informational message.
 * There is a single button that dismisses the message.
 */
public class SBWarningDialog extends SBBasicDialogFragment {
    private final static String CLSS = "SBWarningDialog";


    public SBWarningDialog() {super();}
    public String getDialogType() { return CLSS; }

    public static SBWarningDialog newInstance(String title, String message) {
        SBWarningDialog frag = new SBWarningDialog();
        Bundle args = new Bundle();
        args.putString("title", title);
        args.putString("message", message);
        frag.setArguments(args);
        return frag;
    }
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        Bundle args = getArguments();
        View view = inflater.inflate(R.layout.alert_dialog, container, true);
        View titleView = view.findViewById(R.id.alert_title);
        ((TextView)titleView).setText(args.getString("title"));
        View contentView = view.findViewById(R.id.alert_content);
        ((TextView)contentView).setText(args.getString("message"));


        Button button =(Button) view.findViewById(R.id.dismiss_button);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                dismiss();
            }
        });

        return view;
    }

    /** The system calls this only when creating the layout in a dialog.
     * Override this to modify dialog characteristics
     */
    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        Dialog dialog = super.onCreateDialog(savedInstanceState);
        decorateDialog(dialog);
        return dialog;
    }

}