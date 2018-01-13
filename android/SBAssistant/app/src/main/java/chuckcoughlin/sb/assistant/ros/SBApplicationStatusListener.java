package chuckcoughlin.sb.assistant.ros;


/**
 * Listen for changes in the current application.
 */

public interface SBApplicationStatusListener {
    public void applicationStarted(String appName);
    public void applicationShutdown();
}
