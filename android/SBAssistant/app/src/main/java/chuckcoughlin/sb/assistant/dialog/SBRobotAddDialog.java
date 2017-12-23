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
import android.widget.Toast;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBRosHelper;
import ros.android.util.InvalidRobotDescriptionException;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;

public class SBRobotAddDialog extends SBBasicDialogFragment {
    public static final String CLSS = "SBRobotAddDialog";
    private RobotDescription robot = null;


    public SBRobotAddDialog() {super();}

    public String getDialogType() { return CLSS; }

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
        // Inflate the layout to use as dialog or embedded fragment. dismiss() isn't working
        //LayoutInflater blower = (LayoutInflater)getActivity().getSystemService(getActivity().LAYOUT_INFLATER_SERVICE);
        View view = inflater.inflate(R.layout.discovery_add_dialog, container, true);
        View titleView = view.findViewById(R.id.add_robot_title);
        ((TextView)titleView).setText(R.string.discoveryPopupAddTitle);

        Button button =(Button) view.findViewById(R.id.enter_button);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                editRobotInfo(view);
                resultsMap.put(SBConstants.DIALOG_RESULT,SBConstants.DIALOG_RESULT_ADD);
                handler.handleDialogResult(SBRobotAddDialog.this,resultsMap);
                if( robot!= null ) {
                    resultsMap.put(SBConstants.DIALOG_ROBOT_DESCRIPTION,robot);
                }
                else {
                    resultsMap.put(SBConstants.DIALOG_ERROR,getErrorMessage());
                }
                dismiss();
            }
        });
        button = (Button) view.findViewById(R.id.scan_robot_button);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                //scanRobotClicked(v);
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

        return view;
    }

    /** The system calls this only when creating the layout in a dialog. */
    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        // The only reason you might override this method when using onCreateView() is
        // to modify any dialog characteristics. For example, the dialog includes a
        // title by default, but your custom layout might not need it. So here you can
        // remove the dialog title, but you must call the superclass to get the Dialog.
        this.dialog = super.onCreateDialog(savedInstanceState);
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
            } else {
                data.put("CURL", "");
            }
            if (newWifiName != null && newWifiName.length() > 0) {
                data.put("WIFI", newWifiName);
            } else {
                data.put("WIFI", "");
            }
            if (newWifiPassword != null && newWifiPassword.length() > 0) {
                data.put("WIFIPW", newWifiPassword);
            } else {
                data.put("WIFIPW", "");
            }
            data.put("WIFIENC", "");
            this.robot = createMaster(new RobotId(data));
        }
        else {
            setErrorMessage("Must specify Master URI.");
        }
    }

    private RobotDescription createMaster(RobotId robotId) {
        Log.i(CLSS, "createMaster ["+robotId.toString()+"]");
        RobotDescription newRobot = null;
        SBRosHelper helper = SBRosHelper.getInstance();
        if (robotId != null && robotId.getMasterUri() != null) {
            List<RobotDescription> robotList = helper.getRobots();
            int index = 0;
            while( index<robotList.size() ) {
                RobotDescription robot = robotList.get(index);
                if (robot.getRobotId().equals(robotId)) {
                        Log.i(CLSS, "That robot is already listed.");
                        return newRobot;
                }
                index++;
            }
            Log.i(CLSS, "creating robot description: "+robotId.toString());
            try {
                newRobot = RobotDescription.createUnknown(robotId);
            }
            catch(InvalidRobotDescriptionException irde) {
                setErrorMessage("Invalid robot description: "+irde.getLocalizedMessage());
            }
        }
        else {
            setErrorMessage("Robot Id is null");
        }
        return newRobot;
    }

}
