/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.control;

import android.app.Activity;
import android.speech.tts.TextToSpeech;
import android.util.Log;

import java.util.Locale;

import teleop_service.TwistCommandRequest;

import static chuckcoughlin.sb.assistant.dialog.SBRobotViewDialog.CLSS;

/**
 * Speak a randomly selected message describing an error due to an
 * attempt to crash into a wall.
 */

public class ObstacleErrorAnnunciator extends TextToSpeech {
    private String[][] phrases = {
            // English
            {       "There is a wall ahead. I'm stopping/",
                    "I'm about to hit a wall.",
                    "Stopping due to obstacle.",
                    "I'm stopping before I crash.",
                    "Wall ahead. Back up.",
                    "I cannot proceed. Back up."},
            // Russian
            {},
            // French
            {}
    };

    /**
     * The controller handles the TwistCommand after we update it.
     */
    public ObstacleErrorAnnunciator(Activity activity, TextToSpeech.OnInitListener listener) {
        super(activity,listener);
    }


    /**
     * Convert error text to speach in the chosen language. We are not yet using the distance.
     * @param language index per TwistCommandInterpreter
     * @param distance to obstacle
     */
    public void annunciateError(int language,double distance ){
        int result = 0;
        if( language==TwistCommandController.RUSSIAN ) {
            result =  setLanguage(Locale.forLanguageTag("RU"));
        }
        if( language==TwistCommandController.FRENCH ) {
            result = setLanguage(Locale.FRANCE);
        }
        else {
            result = setLanguage(Locale.UK);
        }
        if( result==TextToSpeech.LANG_NOT_SUPPORTED || result==TextToSpeech.LANG_MISSING_DATA ) {
            Log.w(CLSS,String.format("annunciateError: Language (%d) not available",language));
        }
        else {
            String text = selectText(language);
            speak(text,TextToSpeech.QUEUE_FLUSH,null);
        }
    }

    /**
     * Search for a match in a string containing space-separated tokens.
     * @param language the most recently used language
     * @return a randomly-selected error string in the specified language.
     */
    private String selectText(int language) {
        String[] strings = phrases[language];
        if( strings.length==0 ) strings = phrases[0];  // english
        double rand = Math.random();
        int index = (int)(rand*strings.length);
        return strings[index];
    }
}
