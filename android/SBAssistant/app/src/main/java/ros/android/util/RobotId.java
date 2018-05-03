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

import java.util.Map;

/**
 * Connectivity portion of a RobotDescription.
 * Returned attributes are never null.
 */
public class RobotId implements java.io.Serializable {
    private String masterUri;
    private String deviceName;  // Paired device name
    private String ssid;
    private String wifiPassword;

    public RobotId() {
        this.masterUri = null;
        this.deviceName = null;
        this.ssid = null;
        this.wifiPassword = null;
    }


    public String getMasterUri() {
        return masterUri;
    }
    public String getDeviceName() {
        return (deviceName == null ? "" : deviceName);
    }
    public String getSSID() {
        return (ssid == null ? "" : ssid);
    }
    public String getWifiPassword() {return (wifiPassword == null ? "" : wifiPassword);}

    public void setDeviceName(String name) {this.deviceName = name;}
    public void setMasterUri(String uri) {this.masterUri = uri;}

    public void setSSID(String wifi) {this.ssid = wifi;}
    public void setWifiPassword(String passwd) {this.wifiPassword = passwd;}

    @Override
    public String toString() {
        String str = getMasterUri() == null ? "" : getMasterUri();
        if (getSSID() != null) {
            str = str + " On: " + getSSID();
        }

        return str;
    }

    //TODO: not needed?
    private boolean nullSafeEquals(Object a, Object b) {
        if (a == b) { //Handles case where both are null.
            return true;
        }
        if (a == null || b == null) {
            return false;
        }
        //Non-are null
        return a.equals(b);
    }

    // Two instances are equal if their MasterURI's are equal.
    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof RobotId)) {
            return false;
        }
        RobotId lhs = (RobotId) o;
        return nullSafeEquals(this.masterUri, lhs.masterUri);
    }

    @Override
    public int hashCode() {
        // Start with a non-zero constant.
        int result = 17;

        // Include a hash for each field checked by equals().
        result = 31 * result + (masterUri == null ? 0 : masterUri.hashCode());
        result = 31 * result + (ssid == null ? 0 : ssid.hashCode());
        return result;
    }
}