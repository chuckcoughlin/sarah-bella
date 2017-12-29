package ros.android.appmanager;
/*
 * See: http://docs.ros.org/hydro/api/android_apps/html/PlatformInfoServiceClient_8java_source.html
 */

import android.util.Log;

import org.ros.exception.ServiceNotFoundException;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.master.client.TopicSystemState;
import org.ros.master.client.SystemState;
import org.ros.master.client.MasterStateClient;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import app_manager.Icon;
import ros.android.msgs.PlatformInfo;
import ros.android.msgs.GetPlatformInfo;
import ros.android.msgs.GetPlatformInfoRequest;
import ros.android.msgs.GetPlatformInfoResponse;

public class PlatformInfoServiceClient extends AbstractNodeMain {
    private String namespace; // this is the namespace under which all rapp manager services reside.
    private String robotUniqueName; // unique robot name, simply the above with stripped '/''s.
    private ServiceResponseListener<GetPlatformInfoResponse> platformInfoListener;
    private PlatformInfo platformInfo;
    private ConnectedNode connectedNode;
    private String errorMessage = "";

    public PlatformInfoServiceClient(String namespace) {
        this.namespace = namespace;
        this._createListeners();
    }

    public PlatformInfoServiceClient() { this._createListeners(); }

    private void _createListeners() {
        this.platformInfoListener = new ServiceResponseListener<GetPlatformInfoResponse>() {
            @Override
            public void onSuccess(GetPlatformInfoResponse message) {
                Log.i("ApplicationManagement", "platform info retrieved successfully");
                platformInfo = message.getPlatformInfo();
            }

            @Override
            public void onFailure(RemoteException e) {
                Log.e("ApplicationManagement", "failed to get platform information!");
            }
        };
    }

    public void waitForResponse() throws ServiceNotFoundException {
        int count = 0;
        while ( platformInfo == null ) {
            if ( errorMessage != "" ) {  // errorMessage gets set by an exception in the run method
                throw new ServiceNotFoundException(errorMessage);
            }
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                throw new RosRuntimeException(e);
            }
            if ( count == 20 ) {  // timeout.
                throw new ServiceNotFoundException("timed out waiting for a platform_info service response");
            }
            count = count + 1;
        }
    }

    public PlatformInfo getPlatformInfo() {
        return platformInfo;
    }

    public String getRobotAppManagerNamespace() {
        return this.namespace;
    }

     public String getRobotUniqueName() {
         return this.robotUniqueName;
     }

     public String getRobotType() {
         return this.platformInfo.getRobot();
     }

     public Icon getRobotIcon() {
         return this.platformInfo.getIcon();
     }

     @Override
     public void onStart(final ConnectedNode connectedNode) {
         if (this.connectedNode != null) {
             errorMessage = "service client instances may only ever be executed once";
             Log.e("ApplicationManagement", errorMessage + ".");
             return;
         }
         this.connectedNode = connectedNode;

         // Find the rapp manager namespace
         int count = 0;
         MasterStateClient masterClient = new MasterStateClient(this.connectedNode, this.connectedNode.getMasterUri());
         while ( this.namespace == null ) {
             SystemState systemState = masterClient.getSystemState();
             for (TopicSystemState topic : systemState.getTopics()) {
                 String name = topic.getTopicName();
                 GraphName graph_name = GraphName.of(name);
                 if ( graph_name.getBasename().toString().equals("app_list") ) {
                     this.namespace = graph_name.getParent().toString();
                     this.robotUniqueName = graph_name.getParent().toRelative().toString();
                     Log.i("ApplicationManagement", "found the namespace for the robot app manager [" + this.namespace + "]");
                     break;
                 }
             }
             try {
                 Thread.sleep(200);
             } catch (Exception e) {
                 errorMessage = "interrupted while looking for the robot app manager.";
                 Log.w("ApplicationManagement", errorMessage);
                 return;
             }
             if ( count == 10 ) {  // timeout - 2s.
                 errorMessage = "Timed out waiting for the robot app manager to appear.";
                 Log.w("ApplicationManagement", errorMessage);
                 return;
             }
             count = count + 1;
         }

         // Find the platform information
         NameResolver resolver = this.connectedNode.getResolver().newChild(this.namespace);
         String serviceName = resolver.resolve("platform_info").toString();
         ServiceClient<GetPlatformInfoRequest, GetPlatformInfoResponse> client;
         try {
             client = connectedNode.newServiceClient(serviceName,
                     GetPlatformInfo._TYPE);
             Log.d("ApplicationManagement", "service client created [" + serviceName + "]");
         } catch (ServiceNotFoundException e) {
             errorMessage = "Service not found [" + serviceName + "]";
             Log.w("ApplicationManagement", errorMessage);
             return;
         } catch (RosRuntimeException e) {
             errorMessage = "Couldn't connect to the platform_info service [is ROS_IP set?][" + e.getMessage() + "]";
             Log.e("ApplicationManagement", errorMessage);
             return;
         }
         final GetPlatformInfoRequest request = client.newMessage();
         client.call(request, platformInfoListener);
         Log.d("ApplicationManagement", "service call done [" + serviceName + "]");
     }

     @Override
     public GraphName getDefaultNodeName() {
         return null;
     }
 }
