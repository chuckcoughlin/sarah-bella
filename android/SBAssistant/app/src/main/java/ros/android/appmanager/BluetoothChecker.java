/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * (MIT License)
 */
package ros.android.appmanager;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.util.Log;

import java.util.HashSet;
import java.util.Set;

import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBRobotManager;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;

/**
 * Attempt to create a network connection via Bluetooth.
 * NOTE: Bluetooth is not supported on the emulator. The adapter will be null.
 */
public class BluetoothChecker {
    private final static String CLSS = "BluetoothChecker";
    private CheckerThread checkerThread = null;
    private SBRobotConnectionHandler handler;
    private boolean threadRunning;

    public BluetoothChecker(SBRobotConnectionHandler handler) {
        this.threadRunning = false;
        this.handler = handler;
    }

    // An empty string returned implies success, else an error message.
    // For now we always return an error
    public String bluetoothValid(RobotDescription robot, BluetoothAdapter adapter) {
        String errorMsg = "";
        String device = robot.getRobotId().getDeviceName();
        if( adapter==null ) {
            errorMsg = "No bluetooth network";
        }
        else if( device==null || device.isEmpty() ) {
            errorMsg = "No bluetooth device specified";
        }
        else {
            if( !adapter.isEnabled() ) adapter.enable();
            if( !adapter.isEnabled()) {
                errorMsg = "Bluetooth network is not enabled";
            }
        }
        return errorMsg;
    }

    public void beginChecking(RobotDescription robot, BluetoothManager bmgr) {
        if( this.threadRunning ) {
            Log.i(CLSS, "check already in progress ...");
            return;
        }
        String errMsg = bluetoothValid(robot,bmgr.getAdapter());
        if( errMsg.isEmpty() ) {
            checkerThread = new CheckerThread(robot, bmgr);
            checkerThread.start();
        }
        else {
            handler.handleNetworkError(errMsg);
        }
    }

    public void stopChecking() {
        if (checkerThread != null && checkerThread.isAlive()) {
            checkerThread.interrupt();
        }
    }

    private class CheckerThread extends Thread {
        private BluetoothAdapter adapter;
        private Set<BluetoothDevice> pairedDevices;
        private RobotDescription robot;

        public CheckerThread(RobotDescription rbt, BluetoothManager bmgr) {
            this.robot = rbt;
            this.adapter = bmgr.getAdapter();
            this.pairedDevices = new HashSet<>();
            setDaemon(true);
            // don't require callers to explicitly kill all the old checker threads.
            setUncaughtExceptionHandler(new UncaughtExceptionHandler() {
                @Override
                public void uncaughtException(Thread thread, Throwable ex) {
                    String msg = String.format("Uncaught exception checking Bluetooth Connection: %s",ex.getLocalizedMessage());
                    Log.e(CLSS,msg ,ex);
                    handler.handleNetworkError(msg);
                    threadRunning = false;
                }
            });
        }


        @Override
        public void run() {
            threadRunning = true;
            boolean success = false;
            String errorMsg = "";
            try {
                adapter.startDiscovery();
                int cycle = 0;
                while (cycle < 10 && !success) {
                    pairedDevices = adapter.getBondedDevices();
                    for (BluetoothDevice device : pairedDevices) {
                        Log.i(CLSS, String.format("BluetoothChecker: Found %s %s %s", device.getName(), device.getType(), device.getAddress()));
                        if( device.getName().equalsIgnoreCase(robot.getRobotId().getDeviceName()) ) {
                            success = true;
                            break;
                        }
                    }
                    Thread.sleep(1000);
                    cycle++;
                }
                adapter.cancelDiscovery();
            }
            catch (Throwable ex) {
                Log.e(CLSS, "Exception while searching for Bluetooth for "
                        + robot.getRobotId().getDeviceName(), ex);
                errorMsg = ex.getLocalizedMessage();
            }

            if (success) {
                handler.receiveNetworkConnection();
            }
            else {
                handler.handleNetworkError(errorMsg);
            }
            threadRunning = false;
        }
    }
}