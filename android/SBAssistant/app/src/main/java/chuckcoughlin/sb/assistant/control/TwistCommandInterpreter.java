/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.control;

import android.support.v7.widget.RecyclerView;
import android.view.ViewGroup;
import android.widget.TextView;

import java.util.List;

import teleop_service.TwistCommandRequest;

/**
 * Translates a stream of spoken words in TwistCommand updates.
 * In our word lists end words with a space to avoid false
 * embedded word matches.
 */

public class TwistCommandInterpreter  {
    private final TwistCommandController controller;
    private static final double MIN_VELOCITY = 0.01;
    private String[] backWords = {"back","reverse "};
    private String[] goWords   = {"go ","proceed ","forward "};
    private String[] stopWords = {"stop ","halt "};
    /**
     * The controller handles the TwistCommand after we update it.
     */
    public TwistCommandInterpreter(TwistCommandController c) {
        this.controller =  c;
    }

    /**
     * If the word list can be recognized as a valid command, then
     * the interpreter will issue "commandVelocity" on the controller.
     * @param command current state of navigation
     * @param text space-delimited list of spoken words
     * @return true if we've acted on the contents
     */
    public boolean handleWordList(TwistCommandRequest command, String text){
        String tokens = text.toLowerCase()+" ";   // Can always search for whole words
        boolean success = false;
        // First check for stop words
        if( listContains(tokens,stopWords) ) {
            controller.commandVelocity(0.,0.);
            success = true;
        }
        double velx = command.getLinearX();
        double angz = command.getAngularZ();
        if( listContains(tokens,goWords) ) {
            if( velx < MIN_VELOCITY ) velx = MIN_VELOCITY;
            success = true;
        }
        else if( listContains(tokens,backWords) ) {
            if( velx > -MIN_VELOCITY ) velx = -MIN_VELOCITY;
            success = true;
        }

        // Success means that we've seen enough tokens to operate
        if( success ) {
            controller.commandVelocity(velx,angz);
        }
        return success;
    }

    /**
     *
     * @param a input space-delimited list of raw tokens
     * @param b list of standard tokens
     * @return true if any word from list b appears in a
     */
    private boolean listContains(String a,String[] b) {
        for(String bstring :b) {
            if( a.indexOf(bstring)>-1 ) return true;
        }
        return false;
    }
}
