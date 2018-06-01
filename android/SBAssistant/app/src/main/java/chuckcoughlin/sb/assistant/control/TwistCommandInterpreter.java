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
 *
 *
 */

public class TwistCommandInterpreter  {
    private final TwistCommandController controller;
    private static final double MIN_ANGLE = 0.15;
    private static final double MIN_VELOCITY = 0.01;
    // These are the array indices for the different languages.
    private static int ENGLISH = 0;
    private static int RUSSIAN = 1;
    private String[][] backWords = {{"back","reverse "},{}};
    private String[][] goWords   = {{"go ","proceed ","forward "},{}};
    private String[][] stopWords = {{"stop ","halt ","pause ","paws "},{}};
    private String[][] turnWords = {{"turn "},{}};
    private String[][] turnDirectionWords = {{"right ","left "},{}};
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
     * @return a language constant if we've made a match, else -1
     */
    public int handleWordList(TwistCommandRequest command, String text){
        String tokens = text.toLowerCase()+" ";   // Can always search for whole words
        int result = -1;
        int language = -1;
        // First check for stop words
        if( (language=listContains(tokens,stopWords))>=0 ) {
            controller.commandVelocity(0.,0.);
            result = language;
        }
        double velx = command.getLinearX();
        double angz = command.getAngularZ();
        if( (language=listContains(tokens,goWords))>=0 ) {
            if( velx < MIN_VELOCITY ) velx = MIN_VELOCITY;
            result = language;
        }
        else if( (language=listContains(tokens,backWords))>=0 ) {
            if( velx > -MIN_VELOCITY ) velx = -MIN_VELOCITY;
            result = language;
        }

        // Success means that we've seen enough tokens to operate
        if( result>=0 ) {
            controller.commandVelocity(velx,angz);
        }
        return result;
    }

    /**
     *
     * @param a input space-delimited list of raw tokens
     * @param b list of standard tokens
     * @return an integer corresponding to the language of the match.
     *         -1 indicates no match.
     */
    private int listContains(String a,String[][] b) {
        String testString = "";
        for (int outter = 0; outter < b.length; outter++) {
            for (int inner = 0; inner < b[outter].length; inner++) {
                testString = b[outter][inner];
                if( a.indexOf(testString)>-1 ) return outter;
            }
        }
        return -1;
    }
}
