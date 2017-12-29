package ros.android.appmanager;
/*
 * See: http://docs.ros.org/hydro/api/android_apps/html/StatusServiceClient_8java_source.html
 */
import android.util.Log;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import ros.android.msgs.Constants;
import ros.android.msgs.Status;
import ros.android.msgs.StatusRequest;
import ros.android.msgs.StatusResponse;

public class StatusServiceClient extends AbstractNodeMain {
    private String namespace; // this is the namespace under which all rapp manager services reside.
    private ServiceResponseListener<StatusResponse> listener;
    private ConnectedNode connectedNode;
    // private String applicationNamespace; // namespace under which the rapp manager launches apps.
    private String remoteController = "";
    private Boolean isAvailable = null; // set this to true if it is free OR we are already controlling it.

    public StatusServiceClient(String namespace, final String ourControllerName) {
        this.namespace = namespace;
        this.listener = new ServiceResponseListener<StatusResponse>() {
            @Override
            public void onSuccess(StatusResponse message) {
                //isAvailable = ( message.getRemoteController() == Constants.NO_REMOTE_CONNECTION) ? Boolean.TRUE : Boolean.FALSE;
                if ( message.getRemoteController().equals(Constants.NO_REMOTE_CONNECTION)) {
                    Log.i("ApplicationManagement", "rapp manager is available");
                    isAvailable = Boolean.TRUE;
                } else {
                    if ( ourControllerName.equals(message.getRemoteController())) {
                        Log.i("ApplicationManagement", "rapp manager is already being remote controlled by us");
                        isAvailable = Boolean.TRUE;
                    } else {
                        Log.i("ApplicationManagement", "rapp manager is being remote controlled [" + message.getRemoteController() + "]");
                        isAvailable = Boolean.FALSE;
                    }
                }
            }
            @Override
            public void onFailure(RemoteException e) {
                Log.e("ApplicationManagement", "failed to send status request!");
            }
        };
    }

    public String getRemoteControllerName() { return remoteController; }

    public void waitForResponse() {
        while (isAvailable == null) {
            try {
                Thread.sleep(100);
            } catch (Exception e) {
            }
        }
    }

    public Boolean isAvailable() {
        return isAvailable;
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        if (this.connectedNode != null) {
            Log.e("ApplicationManagement", "service client instances may only ever be executed once.");
            return;
        }
        this.connectedNode = connectedNode;
        NameResolver resolver = this.connectedNode.getResolver().newChild(this.namespace);
        String serviceName = resolver.resolve("status").toString();
        ServiceClient<StatusRequest, StatusResponse> client;
        try {
            Log.d("ApplicationManagement", "service client created [" + serviceName + "]");
            client = connectedNode.newServiceClient(serviceName,
                    Status._TYPE);
        } catch (ServiceNotFoundException e) {
          Log.w("ApplicationManagement", "service not found [" + serviceName + "]");
             throw new RosRuntimeException(e);
         } catch (RosRuntimeException e) {
             Log.e("ApplicationManagement", "failed to create connection to the rapp manager's status service [" + e.getMessage() + "]");
             throw e;
         }
         final StatusRequest request = client.newMessage();
         client.call(request, listener);
         Log.d("ApplicationManagement", "service call done [" + serviceName + "]");
     }

     @Override
     public GraphName getDefaultNodeName() {
         return null;
     }
 }
