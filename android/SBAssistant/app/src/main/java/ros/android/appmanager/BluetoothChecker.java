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
import ros.android.util.RobotId;

/**
 * Attempt to create a network connection via Bluetooth.
 * NOTE: Bluetooth is not supported on the emulator. The adapter will be null.
 */
public class BluetoothChecker {
    private final static String CLSS = "BluetoothChecker";
    private CheckerThread checkerThread = null;
    private SBRobotConnectionHandler handler;
    private String bluetoothError = "";


    public BluetoothChecker(SBRobotConnectionHandler handler) {
        this.handler = handler;
    }

    // An empty string returned implies success, else an error message.
    // For now we always return an error
    public boolean bluetoothValid(BluetoothAdapter adapter,String device) {
        boolean result = false;
        if( adapter==null ) {
            bluetoothError = "No bluetooth network";
        }
        else if( device==null || device.isEmpty() ) {
            bluetoothError = "No bluetooth device specified";
        }
        else {
            if( !adapter.isEnabled() ) adapter.enable();
            if( !adapter.isEnabled()) {
                bluetoothError = "Bluetooth network is not enabled";
                result = false;
            }
            else {
                result = true;
            }
        }
        return result;
    }

    public void beginChecking(RobotId robotId, BluetoothManager bmgr,String device) {
        if(bluetoothValid(bmgr.getAdapter(),device)) {
            checkerThread = new CheckerThread(robotId, bmgr,device);
            checkerThread.start();
        }
        else {
            handler.handleNetworkError(SBConstants.NETWORK_BLUETOOTH,bluetoothError);
        }
    }

    public void stopChecking() {
        if (checkerThread != null && checkerThread.isAlive()) {
            checkerThread.interrupt();
        }
    }

    private class CheckerThread extends Thread {
        private RobotId robotId;
        private BluetoothAdapter adapter;
        private Set<BluetoothDevice> pairedDevices;
        private String deviceName;

        public CheckerThread(RobotId robotId, BluetoothManager bmgr,String targetDevice) {
            this.robotId = robotId;
            this.adapter = bmgr.getAdapter();
            this.pairedDevices = new HashSet<>();
            this.deviceName = targetDevice;
            setDaemon(true);
            // don't require callers to explicitly kill all the old checker threads.
            setUncaughtExceptionHandler(new UncaughtExceptionHandler() {
                @Override
                public void uncaughtException(Thread thread, Throwable ex) {
                    Log.e(CLSS, String.format("Uncaught exception checking Bluetooth Connection: %s",ex.getLocalizedMessage()),ex);
                    handler.handleNetworkError(SBConstants.NETWORK_BLUETOOTH,"Uncaught exception: " + ex.getLocalizedMessage());
                }
            });
        }


        @Override
        public void run() {
            boolean success = false;
            try {
                adapter.startDiscovery();
                int cycle = 0;
                while (cycle < 10 && !success) {
                    pairedDevices = adapter.getBondedDevices();
                    for (BluetoothDevice device : pairedDevices) {
                        Log.i(CLSS, String.format("BluetoothChecker: Found %s %s %s", device.getName(), device.getType(), device.getAddress()));
                        if( device.getName().equalsIgnoreCase(deviceName) ) {
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
                Log.e("RosAndroid", "Exception while searching for Bluetooth for "
                        + robotId.getDeviceName(), ex);
                bluetoothError = ex.getLocalizedMessage();
            }

            if (success) {
                SBRobotManager.getInstance().setDeviceName(deviceName);
                handler.receiveNetworkConnection();
            }
            else {
                handler.handleNetworkError(SBConstants.NETWORK_BLUETOOTH, bluetoothError);
            }
        }
    }
}