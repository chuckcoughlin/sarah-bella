/*
* Copyright (C) 2013 OSRF.
*
* Licensed under the Apache License, Version 2.0 (the "License"); you may not
* use this file except in compliance with the License. You may obtain a copy of
* the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
* License for the specific language governing permissions and limitations under
* the License.
*  
* See: http://docs.ros.org/hydro/api/android_apps/html/AppManager_8java_source.html
*/

package ros.android.appmanager;

import android.util.Log;

import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;

import app_manager.AppList;
import ros.android.msgs.GetAppList;
import ros.android.msgs.GetAppListRequest;
import ros.android.msgs.GetAppListResponse;
import app_manager.StartApp;
import app_manager.StartAppRequest;
import app_manager.StartAppResponse;
import app_manager.StopApp;
import app_manager.StopAppRequest;
import app_manager.StopAppResponse;

public class AppManager extends AbstractNodeMain {

    // unique identifier to key string variables between activities.
    static public final String PACKAGE = "ros.android.appmanager.AppManager";
    private static final String startTopic = "start_app";
    private static final String stopTopic = "stop_app";
    private static final String listService = "list_apps";

    private String appName;
    private NameResolver resolver;
    private ServiceResponseListener<StartAppResponse> startServiceResponseListener;
    private ServiceResponseListener<StopAppResponse>  stopServiceResponseListener;
    private ServiceResponseListener<GetAppListResponse> listServiceResponseListener;
    private MessageListener<AppList> appListListener;
    private Subscriber<AppList> subscriber;

    private ConnectedNode connectedNode;
    private String function = null;

    public AppManager(final String appName, NameResolver resolver) {
        this.appName = appName;
        this.resolver = resolver;
    }

    public AppManager(final String appName) {
        this.appName = appName;
    }

    public AppManager() {

    }

    public void setFunction(String function) {
        this.function = function;
    }

    public void setAppName(String appName) {
        this.appName = appName;
    }

    public void setAppListSubscriber(MessageListener<AppList> appListListener) {
        this.appListListener = appListListener;
    }

    public void setStartService(
            ServiceResponseListener<StartAppResponse> startServiceResponseListener) {
        this.startServiceResponseListener = startServiceResponseListener;
    }

    public void setStopService(
            ServiceResponseListener<StopAppResponse> stopServiceResponseListener) {
        this.stopServiceResponseListener = stopServiceResponseListener;
    }

    public void setListService(
            ServiceResponseListener<GetAppListResponse> listServiceResponseListener) {
        this.listServiceResponseListener = listServiceResponseListener;
    }

    public void continuouslyListApps() {
        subscriber = connectedNode.newSubscriber(resolver.resolve("app_list"), "rocon_app_manager_msgs/AppList");
        subscriber.addMessageListener(this.appListListener);
    }

    public void startApp() {
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
    }

    public void stopApp() {
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
    }

    public void listApps() {
        String listService = resolver.resolve(this.listService).toString();

        ServiceClient<GetAppListRequest, GetAppListResponse> listAppsClient;
        try {
            Log.d("ApplicationManagement", "List app service client created [" + listService + "]");
            listAppsClient = connectedNode.newServiceClient(listService,
                    GetAppList._TYPE);
        } catch (ServiceNotFoundException e) {
            Log.w("ApplicationManagement", "List app service not found [" + listService + "]");
            throw new RosRuntimeException(e);
        }
        final GetAppListRequest request = listAppsClient.newMessage();
        listAppsClient.call(request, listServiceResponseListener);
        Log.d("ApplicationManagement", "List apps service call done [" + listService + "]");
    }

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        if (this.connectedNode != null) {
            Log.e("ApplicationManagement", "app manager instances may only ever be executed once [" + function + "].");
            return;
        }
        this.connectedNode = connectedNode;
        if (function.equals("start")) {
            startApp();
        } else if (function.equals("stop")) {
            stopApp();
        } else if (function.equals("list")) {
            listApps();
        } else if (function.equals("list_apps")) {
            continuouslyListApps();
        }
    }
}
