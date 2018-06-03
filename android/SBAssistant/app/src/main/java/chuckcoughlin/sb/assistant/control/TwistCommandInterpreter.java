/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.control;

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
    private static final double ANGLE_FACTOR = 1.5;       // a factor
    private static final double MAX_ANGLE = 1.0;  // or negative for left
    private static final double SMALLEST_ANGLE = 0.15;
    private static final double VELOCITY_FACTOR = 2.0;    // a factor
    private static final double MAX_VELOCITY = 1.0;      // or negative for backward
    private static final double SMALLEST_VELOCITY = 0.1;
    // Use the TwistCommandController interface to interpret the indices
    // belonging to the different languages:
    // 0 - ENGLISH
    // 1 - RUSSIAN
    // 2 - FRENCH
    private String[][] backWords = {{"back","reverse "},{"перейти назад"},{}};
    private String[][] fastWords   = {{"faster ","not so slow "},{"быстрее","быстро"},{}};
    private String[][] goWords   = {{"go ","proceed ","forward "},{"идите"},{}};
    private String[][] lessWords   = {{"less "},{"менее"},{}};
    private String[][] moreWords   = {{"more "},{"более"},{}};
    private String[][] pivotWords   = {{"pivot ","rotate "},{}};
    private String[][] sharpWords   = {{"sharp"},{"четкость"},{}};
    private String[][] shallowWords   = {{"shallow","not so sharp "},{}};
    private String[][] slowWords   = {{"slower ","not so fast "},{"медленно","медленнее"}};
    private String[][] stopWords = {{"stop ","halt ","pause ","paws "},{"остановить","стоп"}};
    private String[][] turnWords = {{"turn "},{"повернуть"},{"tournez"}};
    private String[][] leftWords = {{"left "},{"налево"},{"gauche"}};
    private String[][] rightWords = {{"right "},{"направо"},{"droite"}};
    /**
     * The controller handles the TwistCommand after we update it.
     */
    public TwistCommandInterpreter(TwistCommandController c) {
        this.controller =  c;
    }

    /**
     * If the word list can be recognized as a valid command, then
     * the interpreter will issue "commandVelocity" on the controller.
     * We return the language that was spoken so that we can respond similarly, if needed.
     * @param command current state of navigation
     * @param text space-delimited list of spoken words
     * @return true if we've made a match
     */
    public boolean handleWordList(TwistCommandRequest command, String text,int language){
        String tokens = text.toLowerCase()+" ";   // Can always search for whole words
        boolean result = false;

        double velx = command.getLinearX();
        double angz = command.getAngularZ();

        // First check for stop words. If we find one, that's aall we need
        if( listContains(tokens,stopWords,language) ) {
            velx = 0.;
            angz = 0.;
            result = true;
        }
        else {
            boolean forward = (velx>=0.);   // Are we going forward or backward?
            boolean turning = false;  // Are we turning

            if( listContains(tokens, goWords, language) ) {
                if (velx < SMALLEST_VELOCITY) velx = SMALLEST_VELOCITY;
                result = true;
                forward= true;
            }
            else if( listContains(tokens, backWords, language) ) {
                if (velx > -SMALLEST_VELOCITY) velx = -SMALLEST_VELOCITY;
                result = true;
                forward = false;
            }
            if( listContains(tokens, fastWords,language) ) {
                velx*= VELOCITY_FACTOR;
                result = true;
            }
            else if( listContains(tokens, slowWords,language) ) {
                velx/= VELOCITY_FACTOR;
                result = true;
            }

            if( listContains(tokens, turnWords,language) ) {
                turning = true;
                result = true;
            }

            if( listContains(tokens, rightWords,language) ) {
                if( angz<= SMALLEST_ANGLE ) angz = SMALLEST_ANGLE;
                turning = true;
                result = true;
            }
            else if( listContains(tokens, leftWords,language) ) {
                if( angz>= -SMALLEST_ANGLE ) angz = -SMALLEST_ANGLE;
                turning = true;
                result = true;
            }

            if( listContains(tokens, sharpWords,language) ) {
                angz*= ANGLE_FACTOR;
                turning = true;
                result = true;
            }
            else if( listContains(tokens, shallowWords,language) ) {
                angz/= ANGLE_FACTOR;
                turning = true;
                result = true;
            }

            // If we're turning, less/more refer to angle, else speed
            if( listContains(tokens, moreWords,language) ) {
                if( turning ) {
                    angz*= ANGLE_FACTOR;
                }
                else {
                    velx*= VELOCITY_FACTOR;
                }
                result = true;
            }
            else if( listContains(tokens,lessWords,language) ) {
                if( turning ) {
                    angz/= ANGLE_FACTOR;
                }
                else {
                    velx/= VELOCITY_FACTOR;
                }
                result = true;
            }

            if( listContains(tokens,pivotWords,language) ) {
                velx = 0.;
                if(angz>=0.) angz+=MAX_ANGLE/2.0;
                else angz-=MAX_ANGLE/2.0;
                result = true;
            }
        }

        // Check bounds
        if( velx>MAX_VELOCITY )     velx = MAX_VELOCITY;
        else if(velx<-MAX_VELOCITY) velx = -MAX_VELOCITY;
        if( angz>MAX_ANGLE )     angz = MAX_ANGLE;
        else if(angz<-MAX_ANGLE) angz = -MAX_ANGLE;

        // Success means that we've seen enough tokens to operate
        if( result ) {
            controller.commandVelocity(velx,angz);
        }
        return result;
    }

    /**
     * Search for a match in a string containing space-separated tokens.
     * @param a input space-delimited list of raw tokens
     * @param b list of standard tokens
     * @param language the top lndex into the word array
     * @return true if an element from the second array is found in the string
     */
    private boolean listContains(String a,String[][] b,int language) {
        String testString = "";
        for (int inner = 0; inner < b[language].length; inner++) {
            testString = b[language][inner];
            if( a.indexOf(testString)>-1 ) return true;
        }
        return false;
    }
}
