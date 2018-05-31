/**
 * Copyright 2017-2018 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.logs;

/**
 * Define an observer of the log list. It will always be the
 * first entry in the list that is removed, if any.
 */
public interface LogListObserver {
  public void notifyLogAppended();
  public void notifyLogChanged();
  public void notifyLogRemoved();
}
