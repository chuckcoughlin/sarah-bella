/**
 * Copyright 2018 Charles Coughlin. All rights reserved.
 * (MIT License)
 */

package chuckcoughlin.sb.assistant.control;


import teleop_service.TwistCommandRequest;

/**
 * The controller handles execution of TwistCommands. The
 * commands are derived from both visual and speech sources.
 */
public interface TwistCommandController {
    // These are the array indices for the different languages.
    public static int ENGLISH = 0;
    public static int RUSSIAN = 1;
    public static int FRENCH = 2;

    /**
     * Notify the controller to execute a twist command.
     * The controller may have some additional checks.
     * @param velx forward speed ~ m/s.
     * @param angz change in direction ~ radians
     */
    public void commandVelocity(double velx,double angz);
    public TwistCommandRequest getCurrentRequest();
    public TwistCommandRequest getTargetRequest();
    /**
     * @return the maximum robot speed in m/src.
     */
    public double getMaxSpeed();
}
