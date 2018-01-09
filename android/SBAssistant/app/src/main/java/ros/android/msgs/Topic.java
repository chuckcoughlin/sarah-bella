package ros.android.msgs;

import org.ros.internal.node.topic.TopicParticipant;
import org.ros.namespace.GraphName;

/**
 * Created by chuckc on 1/9/18.
 */

public class Topic implements TopicParticipant {
    private final GraphName topicName;
    private final String messageType;

    public Topic(GraphName name,String type) {
        this.topicName = name;
        this.messageType = type;
    }

    @Override
    public GraphName getTopicName() { return this.topicName; }

    @Override
    public String getTopicMessageType() { return this.messageType; }

}
