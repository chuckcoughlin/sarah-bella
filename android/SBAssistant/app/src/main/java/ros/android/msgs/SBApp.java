package ros.android.msgs;


import org.ros.internal.message.RawMessage;
import org.ros.internal.message.context.MessageContext;

import java.util.ArrayList;
import java.util.List;

import app_manager.App;
import app_manager.Icon;

/**
 * Ignore icon - associated with the robot, not the application
 * and pairing client - our robots are not paired.
 */

public class SBApp extends BasicMessage implements App {
    private String name;
    private String displayName;
    private String description;
    private String platform;
    private String status;

    public SBApp(MessageContext context) {
        super(context);
        this.name = "";
        this.displayName = "";
        this.description = "";
        this.platform = PlatformInfo.PLATFORM_LINUX;
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

    public Icon getIcon() { return null; }
    public void setIcon(Icon paramIcon) {}
}

