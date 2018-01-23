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

import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import ros.android.msgs.Topic;

/**
 * This class describes an application on the robot. Individual panels are expected to publish
 * and subscribe to appropriate topics when they are informed of the application state.
 */
public class RobotApplication implements java.io.Serializable {
	private static final String CLSS = "RobotApplication";
	// Connection Status
	public static final String APP_STATUS_NOT_RUNNING="Not Running";
	public static final String APP_STATUS_RUNNING = "Running ...";
	public static final String APP_STATUS_UNAVAILABLE = "Unavailable";   // An error


	private static final String NAME_UNKNOWN = "Unknown";
	private static final String TYPE_UNKNOWN = "Unknown";

	private String applicationName;
	private String description;
	private String executionStatus;


	public RobotApplication(String appName, String desc) {
		this.applicationName = appName;
		this.description     = desc;
	}


	public String getApplicationName()  {
		return this.applicationName;
	}
	public String getDescription() {
		return description;
	}

	public String getExecutionStatus() {
		return executionStatus;
	}
	public void setExecutionStatus(String status) {
		this.executionStatus = status;
	}


	@Override
	public boolean equals(Object o) {
		// Return true if the objects are identical.
		if(this == o) {
			return true;
		}
		if(!(o instanceof RobotApplication)) {
			return false;
		}
		RobotApplication lhs = (RobotApplication) o;

		// Applications are the same if they have the same name
		return (applicationName == null ? lhs.applicationName == null : applicationName.equals(lhs.applicationName));
	}

	// Make class Comparable
	@Override
	public int hashCode() {
		// Start with a non-zero constant.
		int result = 17;
		// Include a hash for each field checked by equals().
		result = 31 * result + (applicationName == null ? 0 : applicationName.hashCode());

		return result;
	}
}
