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

import android.util.Log;

import org.ros.exception.RosException;
import org.ros.namespace.GraphName;

import java.net.URI;
import java.util.Date;

import chuckcoughlin.sb.assistant.common.SBConstants;

/**
 * "bean" variable for a robot containing all its properties.
 * Returned properties are never null.
 */
public class RobotDescription implements java.io.Serializable {
	private static final String CLSS = "RobotDescription";


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
	private String applicationName;
	private String platform;
	private Date timeLastSeen;

	public RobotDescription() {
	}

	public RobotDescription(RobotId robotId, String robotName, String robotType, Date timeLastSeen)  {
		setRobotName(robotName);
		setRobotId(robotId);
		this.robotType = robotType;
		this.platform= SBConstants.PLATFORM_LINUX;
		this.timeLastSeen = timeLastSeen;
	}

	public RobotDescription clone() {
		RobotDescription newRobot = new RobotDescription(this.robotId, this.robotName, this.robotType, new Date());
		newRobot.setPlatform(this.platform);
		newRobot.setApplicationName(this.applicationName);
		return newRobot;
	}

	public RobotId getRobotId() {
		return robotId;
	}

	public void setRobotId(RobotId robotId)  {
		this.robotId = robotId;
	}

	public String getRobotName() {
		return (robotName==null?"":robotName);
	}

	public void setRobotName(String robotName)  {
		this.robotName = robotName;
	}

	public String getRobotType() {
		return (robotType==null?"":robotType);
	}
	public void setRobotType(String robotType) {
		this.robotType = robotType;
	}
	public String getApplicationName() {return (applicationName==null?"":applicationName) ;}
	public void setApplicationName(String name) {
		this.applicationName = name;
	}
	public String getPlatform() { return (platform==null?"":platform); }
	public void setPlatform(String plat) { this.platform = plat; }

	public Date getTimeLastSeen() {
		return timeLastSeen;
	}
	public void setTimeLastSeen(Date timeLastSeen) {
		this.timeLastSeen = timeLastSeen;
	}

	public boolean isUnknown() {
		return this.robotName.equals(NAME_UNKNOWN);
	}

	public static RobotDescription createUnknown(RobotId robotId)  {
		return new RobotDescription(robotId, NAME_UNKNOWN, TYPE_UNKNOWN, new Date());
	}

	@Override
	public boolean equals(Object o) {
		if(this == o) {
			return true;
		}
		if(!(o instanceof RobotDescription)) {
			return false;
		}
		RobotDescription lhs = (RobotDescription) o;
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
