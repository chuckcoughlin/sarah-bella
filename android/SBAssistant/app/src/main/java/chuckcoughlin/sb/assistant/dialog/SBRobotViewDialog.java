/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * Code derived from https://developer.android.com/guide/topics/ui/dialogs.html
 * (MIT License)
 */
package chuckcoughlin.sb.assistant.dialog;

import android.app.Dialog;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.ros.SBRosManager;
import ros.android.util.RobotDescription;

public class SBRobotViewDialog extends SBBasicDialogFragment {
    public static final String CLSS = "SBRobotViewDialog";
    private SBRosManager rosManager;
    private RobotDescription robot = null;

    public SBRobotViewDialog() {super();}
    public String getDialogType() { return CLSS; }

    @Override
    public void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        this.rosManager= SBRosManager.getInstance();
        this.robot = rosManager.getRobot();
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.discovery_view_dialog, container, true);
        View titleView = view.findViewById(R.id.view_robot_title);
        ((TextView)titleView).setText(R.string.discoveryViewPopupTitle);

        // No need to inform the caller, just go away
        Button button = (Button) view.findViewById(R.id.dismiss_button);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i(CLSS, "Dismissing dialog "+getDialogType());
                dismiss();
            }
        });

        // Initialize dialog (read-only) fields
        if( robot!=null ) {
            TextView nameField = (TextView) view.findViewById(R.id.robot_name);
            nameField.setText(robot.getRobotName());
            TextView uriField = (TextView) view.findViewById(R.id.uri_editor);
            uriField.setText(robot.getRobotId().getMasterUri());;
            TextView wifiNameField = (TextView) view.findViewById(R.id.wifi_name_editor);
            wifiNameField.setText(robot.getRobotId().getSSID());
            TextView applicationField = (TextView) view.findViewById(R.id.robot_application_editor);
            applicationField.setText(robot.getApplicationName());
        }
        return view;
    }

    /** The system calls this only when creating the dialog layout.
     * Overriden to modify dialog contents.
     */
    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        Dialog dialog = super.onCreateDialog(savedInstanceState);
        decorateDialog(dialog);
        return dialog;
    }
}
