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
    /**
     * The TwistCommand contains current state of the navigation.
     * @return A twist command shared with the controller.
     */
    public TwistCommandRequest getTwistCommand();
    /**
     * Notify the controller to execute a twist command.
     * The controller may have some additional checks.
     * @param velx forward speed ~ m/s.
     * @param angz change in direction ~ radians
     */
    public void commandVelocity(double velx,double angz);
    /**
     * @return the maximum robot speed in m/src.
     */
    public double getMaxSpeed();
}
