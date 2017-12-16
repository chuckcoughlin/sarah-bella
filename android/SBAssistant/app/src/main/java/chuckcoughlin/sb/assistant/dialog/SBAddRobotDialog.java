/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * Code derived from https://developer.android.com/guide/topics/ui/dialogs.html
 * (MIT License)
 */
package chuckcoughlin.sb.assistant.dialog;

import android.app.Dialog;
import android.os.Bundle;
import android.support.v4.app.DialogFragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.Window;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.ros.SBRosHelper;
import ros.android.util.InvalidRobotDescriptionException;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;

public class SBAddRobotDialog extends DialogFragment {
    private static final String CLSS = "SBAddRobotDialog";
    private String title = "no title";

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        int style = DialogFragment.STYLE_NORMAL;
        int theme = android.R.style.Theme_Black_NoTitleBar;
        setStyle(style, theme);
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout to use as dialog or embedded fragment
        View view = inflater.inflate(R.layout.add_uri_dialog, container, false);
        View titleView = view.findViewById(R.id.add_robot_title);
        ((TextView)titleView).setText(title);


        Button button =(Button) view.findViewById(R.id.enter_button);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                editRobotInfo(view);
            }
        });
        button = (Button) view.findViewById(R.id.scan_robot_button);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                //scanRobotClicked(v);
            }
        });
        button = (Button) view.findViewById(R.id.cancel_button);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                dismiss();
            }
        });

        return view;
    }

    /** The system calls this only when creating the layout in a dialog. */
    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        // The only reason you might override this method when using onCreateView() is
        // to modify any dialog characteristics. For example, the dialog includes a
        // title by default, but your custom layout might not need it. So here you can
        // remove the dialog title, but you must call the superclass to get the Dialog.
        Dialog dialog = super.onCreateDialog(savedInstanceState);
        dialog.requestWindowFeature(Window.FEATURE_NO_TITLE);
        return dialog;
    }

    private void editRobotInfo(View parent) {
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
            data.put("URL", newMasterUri);
            if (newControlUri != null && newControlUri.length() > 0) {
                data.put("CURL", newControlUri);
            }
            if (newWifiName != null && newWifiName.length() > 0) {
                data.put("WIFI", newWifiName);
            }
            if (newWifiPassword != null && newWifiPassword.length() > 0) {
                data.put("WIFIPW", newWifiPassword);
            }
            try {
                addMaster(new RobotId(data));
            } catch (InvalidRobotDescriptionException e) {
                Log.i(CLSS, "Invalid Parameters.");
            }
        }
        else {
            Log.i(CLSS, "Must specify Master URI.");
        }
    }
    private void addMaster(RobotId robotId) throws InvalidRobotDescriptionException {
        addMaster(robotId, false);
    }

    private void addMaster(RobotId robotId, boolean connectToDuplicates) throws InvalidRobotDescriptionException {
        Log.i(CLSS, "addMaster ["+robotId.toString()+"]");
        SBRosHelper helper = SBRosHelper.getInstance();
        if (robotId == null || robotId.getMasterUri() == null) {
            throw new InvalidRobotDescriptionException("Empty master URI");
        }
        else {
            List<RobotDescription> robotList = helper.getRobots();
            int index = 0;
            while( index<robotList.size() ) {
                RobotDescription robot = robotList.get(index);
                if (robot.getRobotId().equals(robotId)) {
                    if (connectToDuplicates) {
                        SBRosHelper.getInstance().setCurrentRobot(index);
                        return;
                    }
                    else {
                        Log.i(CLSS, "That robot is already listed.");
                        return;
                    }
                }
                index++;
            }
            Log.i(CLSS, "creating robot description: "+robotId.toString());
            helper.addRobot(RobotDescription.createUnknown(robotId));
            Log.i(CLSS, "description created");
        }
    }

    public void setTitle(String newTitle) { this.title = newTitle; }

}
