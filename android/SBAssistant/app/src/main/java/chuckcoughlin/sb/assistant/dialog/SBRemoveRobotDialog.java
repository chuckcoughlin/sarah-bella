/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *            (MIT License)
 */
package chuckcoughlin.sb.assistant.dialog;


import android.app.Activity;
import android.content.DialogInterface;
import android.support.v7.app.AlertDialog;
import android.text.Spannable;
import android.text.style.ForegroundColorSpan;
import android.text.style.RelativeSizeSpan;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import chuckcoughlin.sb.assistant.ros.SBRosHelper;
import ros.android.util.RobotDescription;

/**
 * Wraps the alert dialog to display a list.
 */
public class SBRemoveRobotDialog {
    private int state;
    private AlertDialog dialog;
    private Activity context;

    public SBRemoveRobotDialog(Activity context) {
        state = 0;
        this.context = context;
        AlertDialog.Builder builder = new AlertDialog.Builder(context);
        SBRosHelper rosHelper = SBRosHelper.getInstance();
        List<RobotDescription> robots = rosHelper.getRobots();
        String newline = System.getProperty("line.separator");
        if (robots.size()>0) {
            boolean[] selections = new boolean[robots.size()];
            Spannable[] robot_names = new Spannable[robots.size()];
            Spannable name;
            for (int i=0; i<robots.size(); i++) {
                name = Spannable.Factory.getInstance().newSpannable(robots.get(i).getRobotName() + newline + robots.get(i).getRobotId());
                name.setSpan(new ForegroundColorSpan(0xff888888), robots.get(i).getRobotName().length(), name.length(), Spannable.SPAN_EXCLUSIVE_EXCLUSIVE);
                name.setSpan(new RelativeSizeSpan(0.8f), robots.get(i).getRobotName().length(), name.length(), Spannable.SPAN_EXCLUSIVE_EXCLUSIVE);
                robot_names[i] = name;
            }
            builder.setTitle("Delete a robot");
            builder.setMultiChoiceItems(robot_names, selections, new DialogSelectionClickHandler());
            builder.setPositiveButton( "Delete Selections", new DialogButtonClickHandler() );
            builder.setNegativeButton( "Cancel", new DialogButtonClickHandler() );
            dialog = builder.create();
        }
        else {
            builder.setTitle("No robots to delete.");
            dialog = builder.create();
            final Timer t = new Timer();
            t.schedule(new TimerTask() {
                public void run() {
                    dismiss();
                }
            }, 3*1000);
        }
    }

    public SBRemoveRobotDialog(Activity context, AlertDialog.Builder builder, String okButton) {
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
            } catch (Exception e) {
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

    public class DialogSelectionClickHandler implements DialogInterface.OnMultiChoiceClickListener {
        public void onClick( DialogInterface dialog, int clicked, boolean selected ) {
            return;
        }
    }

    public class DialogButtonClickHandler implements DialogInterface.OnClickListener {
        public void onClick( DialogInterface dialog, int clicked ) {
            switch( clicked ) {
                case DialogInterface.BUTTON_POSITIVE:
                    dismiss();
                    break;
                case DialogInterface.BUTTON_NEGATIVE:
                    dismiss();
                    break;
            }
        }
    }
}