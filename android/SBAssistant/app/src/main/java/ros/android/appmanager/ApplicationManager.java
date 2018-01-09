/*
* Copyright (C) 2017 Chuck Coughlin
*  (MIT License)
*/

package ros.android.appmanager;

import android.util.Log;

import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import ros.android.util.RobotApplication;


/**
 * For the currently selected application, manage the setup of all
 * publishers and subscribers for the topics associated with that
 * application.
 *
 * NOTE: Consider this being a TopicParticipantManager
 * See: PlatformInfoServiceClient as an aexample.
 */

public class ApplicationManager extends AbstractNodeMain {
    private final static String CLSS = "ApplicationManager";
    private RobotApplication currentApplication;
    private NameResolver resolver;
    private ConnectedNode connectedNode;

    public ApplicationManager(NameResolver resolver) {
        this.currentApplication = null;
        this.resolver = resolver;
    }

    /**
     * Change the current application. If there was a previous application,
     * shutdown any existing listeners, stop subscriptions before re-starting.
     * @param app the new current application.
     */
    public void setApplication(RobotApplication app) {

        this.currentApplication = app;
    }




    public void startApp() {
        /*
        String startTopic = resolver.resolve(this.startTopic).toString();

        ServiceClient<StartAppRequest, StartAppResponse> startAppClient;
        try {
            Log.d("ApplicationManagement", "start app service client created [" + startTopic + "]");
            startAppClient = connectedNode.newServiceClient(startTopic,
                    StartApp._TYPE);
        } catch (ServiceNotFoundException e) {
            Log.w("ApplicationManagement", "start app service not found [" + startTopic + "]");
            throw new RosRuntimeException(e);
        }
        final StartAppRequest request = startAppClient.newMessage();
        request.setName(appName);
        startAppClient.call(request, startServiceResponseListener);
        Log.d("ApplicationManagement", "start app service call done [" + startTopic + "]");
        */
    }

    public void stopApp() {
        /*
        String stopTopic = resolver.resolve(this.stopTopic).toString();

        ServiceClient<StopAppRequest, StopAppResponse> stopAppClient;
        try {
            Log.d("ApplicationManagement", "Stop app service client created");
            stopAppClient = connectedNode.newServiceClient(stopTopic,
                    StopApp._TYPE);
        } catch (ServiceNotFoundException e) {
            Log.w("ApplicationManagement", "Stop app service not found");
            throw new RosRuntimeException(e);
        }
        final StopAppRequest request = stopAppClient.newMessage();
        // request.setName(appName); // stop app name unused for now
        stopAppClient.call(request, stopServiceResponseListener);
        Log.d("ApplicationManagement", "Stop app service call done");
        */
    }


    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }

    /**
     * On start we don't do anything. We wait for the user action of selecting an application.
     * @param connectedNode
     */
    @Override
    public void onStart(final ConnectedNode connectedNode) {
        if (this.connectedNode != null) {
            Log.e(CLSS, "appliation manager instances may only be executed once.");
            return;
        }
        this.connectedNode = connectedNode;
    }
}
