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
import android.widget.EditText;
import android.widget.TextView;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBRosManager;
import ros.android.util.InvalidRobotDescriptionException;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;

public class SBRobotCreateDialog extends SBBasicDialogFragment {
    public static final String CLSS = "SBRobotCreateDialog";
    private SBRosManager rosManager;
    private RobotDescription robot = null;

    public SBRobotCreateDialog() {super();}
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
        View view = inflater.inflate(R.layout.discovery_add_dialog, container, true);
        View titleView = view.findViewById(R.id.add_robot_title);
        ((TextView)titleView).setText(R.string.discoveryPopupDefineTitle);


        Button button =(Button) view.findViewById(R.id.enter_button);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                editRobotInfo(view);
                SBRobotCreateDialog dfrag = SBRobotCreateDialog.this;
                dfrag.setSelectedButton(SBConstants.DIALOG_BUTTON_ADD);

                if( robot!= null ) {
                    Log.i(CLSS,String.format("enter_button: %s %s",dfrag.getSelectedButton(),robot.getRobotName()));
                    dfrag.setPayload(robot);
                }
                handler.handleDialogResult(dfrag);
                dismiss();
            }
        });
        button = (Button) view.findViewById(R.id.scan_robot_button);
        // Scan a QR code
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                //validateRobotClicked(v);
            }
        });
        // No need to inform the caller, just go away
        button = (Button) view.findViewById(R.id.cancel_button);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i(CLSS, "Dismissing dialog "+getDialogType());
                dismiss();
            }
        });

        // Initialize edit fields
        if( robot!=null ) {
            EditText nameField = (EditText) view.findViewById(R.id.robot_name);
            nameField.setText(robot.getRobotName());
            EditText uriField = (EditText) view.findViewById(R.id.uri_editor);
            uriField.setText(robot.getRobotId().getMasterUri());;
            EditText controlUriField = (EditText) view.findViewById(R.id.control_uri_editor);
            controlUriField.setText(robot.getRobotId().getControlUri());
            EditText wifiNameField = (EditText) view.findViewById(R.id.wifi_name_editor);
            wifiNameField.setText(robot.getRobotId().getWifi());
            EditText wifiPasswordField = (EditText) view.findViewById(R.id.wifi_password_editor);
            wifiPasswordField.setText(robot.getRobotId().getWifiPassword());
        }
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

    private void editRobotInfo(View parent) {
        EditText nameField = (EditText) parent.findViewById(R.id.robot_name);
        String name = nameField.getText().toString();
        EditText uriField = (EditText) parent.findViewById(R.id.uri_editor);
        String newMasterUri = uriField.getText().toString();
        EditText controlUriField = (EditText) parent.findViewById(R.id.control_uri_editor);
        String newControlUri = controlUriField.getText().toString();
        EditText wifiNameField = (EditText) parent.findViewById(R.id.wifi_name_editor);
        String newWifiName = wifiNameField.getText().toString();
        EditText wifiPasswordField = (EditText) parent.findViewById(R.id.wifi_password_editor);
        String newWifiPassword = wifiPasswordField.getText().toString();
        if (newMasterUri != null && newMasterUri.length() > 0) {
            Map<String, Object> data = new HashMap<String, Object>();
            data.put("URI", newMasterUri);
            if (newControlUri != null && newControlUri.length() > 0) {
                data.put("CURL", newControlUri);
            }
            else {
                data.put("CURL", "");
            }
            if (newWifiName != null && newWifiName.length() > 0) {
                data.put("WIFI", newWifiName);
            }
            else {
                data.put("WIFI", "");
            }
            if (newWifiPassword != null && newWifiPassword.length() > 0) {
                data.put("WIFIPW", newWifiPassword);
            }
            else {
                data.put("WIFIPW", "");
            }
            data.put("WIFIENC", "");

            this.robot = createMaster(name,new RobotId(data));
            Log.i(CLSS,"editRobotInfo: created master "+name);
        }
        else {
            Log.w(CLSS,"editRobotInfo: No master URI specified");
            setErrorMessage("Must specify Master URI.");
        }
    }

    /**
     * Create or update a robot from the supplied data, if valid.
     * @param name new name from the form
     * @param robotId id created from form fields
     * @return new current robot description
     */
    private RobotDescription createMaster(String name,RobotId robotId) {
        Log.i(CLSS, "createMaster ["+robotId.toString()+"]");

        RobotDescription newRobot = null;
        if (robotId != null && robotId.getMasterUri() != null) {
            if( robot==null  ) {
                // Create a new robot
                try {
                    newRobot = RobotDescription.createUnknown(robotId);
                    newRobot.setRobotName(name);
                }
                catch (InvalidRobotDescriptionException irde) {
                    Log.w(CLSS, String.format("Invalid robot description: %s",irde.getLocalizedMessage()));
                    setErrorMessage("Invalid robot description: " + irde.getLocalizedMessage());
                }
            }
            else {
                // Update the current robot
                newRobot = robot.clone();
                newRobot.setRobotName(name);
                newRobot.setRobotId(robotId);
            }
        }
        else {
            Log.w(CLSS, "WARNING: no robotId supplied");
            setErrorMessage("A valid robot Id is required");
        }
        return newRobot;
    }

}
