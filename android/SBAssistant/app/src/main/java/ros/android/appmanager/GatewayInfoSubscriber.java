package ros.android.appmanager;
/*
 * An example of a message subscriber - See MasterChecker
 * See: http://docs.ros.org/hydro/api/android_apps/html/GatewayInfoSubscriber_8java_source.html
 */

import android.util.Log;

import org.ros.exception.RosRuntimeException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import gateway_msgs.GatewayInfo;

public class GatewayInfoSubscriber extends AbstractNodeMain {
    private MessageListener<GatewayInfo> listener;
    private ConnectedNode connectedNode;
    private Subscriber<GatewayInfo> subscriber;
    private NameResolver resolver;
    private String gatewayName = "";
    private String errorMessage = "";

    public GatewayInfoSubscriber() {
    }

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }

    public String getGatewayName() {
        return gatewayName;
    }

    public void waitForResponse() throws RosRuntimeException {
        int count = 0;
        while (this.gatewayName == "") {
            if (errorMessage != "") {  // errorMessage gets set by an exception in the run method
                throw new RosRuntimeException(errorMessage);
            }
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                throw new RosRuntimeException(e);
            }
            if (count == 20) {  // timeout.
                Log.e("ApplicationManagement", "timed out waiting for a gateway_info publication");
                throw new RosRuntimeException("timed out waiting for a gateway_info publication");
            }
            count = count + 1;
        }
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        if (this.connectedNode != null) {
            Log.e("ApplicationManagement", "gateway info subscribers may only ever be executed once.");
            return;
        }
        this.connectedNode = connectedNode;
        NameResolver resolver = this.connectedNode.getResolver().newChild("gateway");
        String topicName = resolver.resolve("gateway_info").toString();
        subscriber = connectedNode.newSubscriber(topicName, "gateway_msgs/GatewayInfo");
        this.listener = new MessageListener<GatewayInfo>() {
            @Override
            public void onNewMessage(GatewayInfo message) {
                gatewayName = message.getName();
                Log.i("ApplicationManagement", "gateway info retrieved successfully [" + gatewayName + "]");
            }
        };
        subscriber.addMessageListener(this.listener);
        Log.d("ApplicationManagement", "latched gateway info subscriber created [" + topicName + "]");
    }
}
