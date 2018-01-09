/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *    
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package ros.android.util;

import java.util.Date;

import app_manager.Icon;
import ros.android.msgs.PlatformInfo;

/**
 * This class describes an application on the robot. An application consists
 * of a set of published and subscribed message topics. This is the Java
 * equivalent of a database entry.
 */
public class RobotApplication implements java.io.Serializable {
	private static final String CLSS = "RobotApplication";
	// Connection Status
	public static final String CONNECTION_STATUS_UNCONNECTED="UNCONNECTED";
	public static final String CONNECTION_STATUS_CONNECTING = "CONNECTING ...";
	public static final String CONNECTION_STATUS_CONNECTED  = "CONNECTED";
	public static final String CONNECTION_STATUS_UNAVAILABLE = "UNAVAILABLE";   // An error

	public static final String OK = "ok";
	public static final String ERROR = "exception";
	public static final String WIFI = "invalid wifi";
	public static final String CONTROL = "not started";

	private static final String NAME_UNKNOWN = "Unknown";
	private static final String TYPE_UNKNOWN = "Unknown";

	private static final long serialVersionUID = 1L;
	private RobotId robotId;
	private String robotName;
	private String robotType;
	private Icon robotIcon;
	private String platformType;
	private String gatewayName;
	private String connectionStatus;
	private Date timeLastSeen;

	public RobotApplication() {
	}

	public RobotApplication(RobotId robotId, String robotName, String robotType, Icon robotIcon, String gatewayName, Date timeLastSeen) {
		this(robotId,robotName,robotType,timeLastSeen);
		this.robotType = robotType;
		this.robotIcon = robotIcon;
		this.gatewayName = gatewayName;
		this.timeLastSeen = timeLastSeen;
	}

	public RobotApplication(RobotId robotId, String robotName, String robotType, Date timeLastSeen)  {
		setRobotName(robotName);
		setRobotId(robotId);
		this.robotType = robotType;
		this.robotIcon = null;
		this.gatewayName = "192.168.0.1";
		this.platformType= PlatformInfo.PLATFORM_LINUX;
		this.connectionStatus = CONNECTION_STATUS_UNCONNECTED;
		this.timeLastSeen = timeLastSeen;
	}

	public RobotApplication clone() {
		RobotApplication newRobot = new RobotApplication(this.robotId, this.robotName, this.robotType, new Date());
		return newRobot;
	}

	public RobotId getRobotId() {
		return robotId;
	}

	public void setRobotId(RobotId robotId)  {
		this.robotId = robotId;
	}

	public String getRobotName() {
		return robotName;
	}

	public void setRobotName(String robotName)  {
		this.robotName = robotName;
	}

	public String getRobotType() {
		return robotType;
	}
	public void setRobotType(String robotType) {
		this.robotType = robotType;
	}
	public String getGatewayName() { return this.gatewayName; }
	public void setGatewayName(String name) { this.gatewayName = name; }
	public String getPlatformType() { return this.platformType; }
	public void setPlatformType(String type) { this.platformType = type; }
	public Icon getRobotIcon() { return this.robotIcon; }
	public void setRobotIcon(Icon icon) { this.robotIcon=icon; }

	public String getConnectionStatus() {
		return connectionStatus;
	}
	public void setConnectionStatus(String connectionStatus) {
		this.connectionStatus = connectionStatus;
	}

	public Date getTimeLastSeen() {
		return timeLastSeen;
	}
	public void setTimeLastSeen(Date timeLastSeen) {
		this.timeLastSeen = timeLastSeen;
	}

	public boolean isUnknown() {
		return this.robotName.equals(NAME_UNKNOWN);
	}

	public static RobotApplication createUnknown(RobotId robotId) throws InvalidRobotDescriptionException {
		return new RobotApplication(robotId, NAME_UNKNOWN, TYPE_UNKNOWN, new Date());
	}

	@Override
	public boolean equals(Object o) {
		// Return true if the objects are identical.
		// (This is just an optimization, not required for correctness.)
		if(this == o) {
			return true;
		}

		// Return false if the other object has the wrong type.
		// This type may be an interface depending on the interface's
		// specification.
		if(!(o instanceof RobotApplication)) {
			return false;
		}

		// Cast to the appropriate type.
		// This will succeed because of the instanceof, and lets us access
		// private fields.
		RobotApplication lhs = (RobotApplication) o;

		// Check each field. Primitive fields, reference fields, and nullable
		// reference
		// fields are all treated differently.
		return (robotId == null ? lhs.robotId == null : robotId.equals(lhs.robotId));
	}

	// I need to override equals() so I'm also overriding hashCode() to match.
	@Override
	public int hashCode() {
		// Start with a non-zero constant.
		int result = 17;

		// Include a hash for each field checked by equals().
		result = 31 * result + (robotId == null ? 0 : robotId.hashCode());

		return result;
	}
}
