package ros.android.msgs;


import org.ros.internal.message.RawMessage;
import org.ros.internal.message.context.MessageContext;

import java.util.ArrayList;
import java.util.List;

import chuckcoughlin.sb.assistant.common.SBConstants;

/**
 * A concrete reprepresentation of the sb_status topic.
 */

public class SBStatus extends BasicMessage {
    private String name;
    private String displayName;
    private String description;
    private String platform;
    private String status;

    public SBStatus(MessageContext context) {
        super(context);
        this.name = "";
        this.displayName = "";
        this.description = "";
        this.platform = SBConstants.PLATFORM_LINUX;
        this.status = "";
    }

    public String getName() {return this.name; }
    public void setName(String paramString) { this.name = paramString; }

    public String getDisplayName() { return this.displayName; }
    public void setDisplayName(String paramString) { this.displayName = paramString; }

    public String getDescription() { return this.description; }
    public void setDescription(String paramString) { this.description = paramString; }

    public String getPlatform() { return this.platform; }
    public void setPlatform(String paramString) { this.platform = paramString; }

    public String getStatus() { return this.status; }
    public void setStatus(String paramString) { this.status = paramString; }

}

