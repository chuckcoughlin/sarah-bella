/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * (MIT License)
 * This is a generic class that handles message subscription
 * startup and shutdown.
 */

package chuckcoughlin.sb.assistant.common;


import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

/**
 * Create a model class for various lists - a name/value pair.
 * Add a hint to help the user.
 */

public abstract class AbstractMessageListener<T> implements MessageListener<T> {
    Subscriber<T> subscriber = null;
    private final String messageType;

    public AbstractMessageListener(String type) {
        this.messageType = type;
    }

    public void subscribe(ConnectedNode node,String topic) {
        if( node!=null ) {
            this.subscriber = node.newSubscriber(topic,messageType);
            subscriber.addMessageListener(this);
        }
    }

    public void shutdown() {
        if(subscriber!=null) subscriber.removeMessageListener(this);
    }

    public abstract void onNewMessage (T msg );
}
