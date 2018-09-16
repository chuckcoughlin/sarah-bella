/*
 * Copyright 2018 Charles Coughlin. All rights reserved.
 * (MIT License)
 *
 * Code was derived from Google (Apache License) and heavily modified to separate control and graphics code.
 * @See: https://github.com/rosjava/android_core/tree/kinetic/android_15/src/org/ros/android/view
 */

package org.ros.android.view;

import android.content.Context;
import android.graphics.Point;
import android.util.AttributeSet;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.animation.Animation;
import android.view.animation.Animation.AnimationListener;
import android.view.animation.AnimationSet;
import android.view.animation.LinearInterpolator;
import android.view.animation.RotateAnimation;
import android.view.animation.ScaleAnimation;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.TextView;

import java.util.Timer;
import java.util.TimerTask;

import audio_locator.AudioLocation;
import chuckcoughlin.sb.assistant.R;


/**
 * Based on DirectControlstickView, this class flashes a wedge in the direction of an acoustic "event".
 * - All coordinates are with respect to the direction in which the robot is currently facing and
 *       its current position (there is no use of Odometry). Animation simply drains the color to black.
 * - Display only, there is no control as a result of this screen.
 *
 */
public class AudioCompassView extends RelativeLayout implements AnimationListener {
    private static final String CLSS = "AudioCompassView";

    // ============================= Graphic Constants ============================================
    /**
     * BOX_TO_CIRCLE_RATIO The dimensions of the square box that contains the
     * circle, rings, etc are 300x300. The circles, rings, etc have a diameter of
     * 220. The ratio of the box to the circles is 300/220 = 1.363636. This ratio
     * stays the same regardless of the size of the virtual compass.
     */
    private static final float BOX_TO_CIRCLE_RATIO = 1.363636f;
    /**
     * ORIENTATION_TACK_FADE_RANGE The range in degrees around the current
     * orientation where the {@link #orientationWidget}s will be visible.
     * `
     */
    private static final float ORIENTATION_TACK_FADE_RANGE = 40.0f;
    /**
     * FADE_TO_BLACK_DELAY Time (in milliseconds) to wait before
     * fading the yellow event signal to background.
     */
    private static final long FADE_TO_BLACK_DELAY = 200L;
    private static final float WEDGE_SIZE = 15.f;  // Degrees
    // ============================================================================================
    private static final float FLOAT_EPSILON = 0.001f;   // For float equality
    private static final int INVALID_POINTER_ID = -1;


    private ImageView intensity; // intensity circle used to show the current magnitude.

    /**
     * wedgeRadius The center coordinates of the parent layout holding all the
     * elements of the virtual joystick. The coordinates are relative to the
     * immediate parent (mainLayout). Since the parent must be a square centerX =
     * centerY = radius.
     */
    private float wedgeRadius = Float.NaN;
    /**
     * magnitudeIndicator Shows the current linear velocity as a percent value.
     * This TextView will be on the opposite side of the contact to ensure that is
     * it visible most of the time. The font size and distance from the center of
     * the widget are automatically computed based on the size of parent
     * container.
     */
    private TextView magnitudeText;
    private RelativeLayout mainLayout; // Parent layout containing all elements of the compass.
    /**
     * orientationWidget 4 long tacks on the major axes and 20 small tacks off of
     * the major axes at 15 degree increments. These fade in and fade out to
     * collectively indicate the current orientation.
     */
    private ImageView[] orientationWidget;
    /**
     * parentSize The length (width==height ideally) of a side of the parent
     * container that holds the virtual joystick.
     */
    private float parentSize = Float.NaN;


    public AudioCompassView(Context context) {
        super(context);
        initCompass(context);
    }

    public AudioCompassView(Context context, AttributeSet attrs) {
        super(context, attrs);
        initCompass(context);
    }

    public AudioCompassView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        initCompass(context);
    }


    /**
     * Sets up the visual elements of the compass.
     */
    private void initCompass(Context context) {
        // All the virtual joystick elements must be centered on the parent.
        setGravity(Gravity.CENTER);
        // Instantiate the elements from the layout XML file.
        LayoutInflater.from(context).inflate(R.layout.audio_compass, this, true);
        mainLayout = (RelativeLayout) findViewById(R.id.audio_compass_layout);
        magnitudeText = (TextView) findViewById(R.id.magnitude);
        intensity = (ImageView) findViewById(R.id.intensity);
        orientationWidget = new ImageView[24];
        orientationWidget[0] = (ImageView) findViewById(R.id.wedge_0_degrees);
        orientationWidget[1] = (ImageView) findViewById(R.id.wedge_15_degrees);
        orientationWidget[2] = (ImageView) findViewById(R.id.wedge_30_degrees);
        orientationWidget[3] = (ImageView) findViewById(R.id.wedge_45_degrees);
        orientationWidget[4] = (ImageView) findViewById(R.id.wedge_60_degrees);
        orientationWidget[5] = (ImageView) findViewById(R.id.wedge_75_degrees);
        orientationWidget[6] = (ImageView) findViewById(R.id.wedge_90_degrees);
        orientationWidget[7] = (ImageView) findViewById(R.id.wedge_105_degrees);
        orientationWidget[8] = (ImageView) findViewById(R.id.wedge_120_degrees);
        orientationWidget[9] = (ImageView) findViewById(R.id.wedge_135_degrees);
        orientationWidget[10] = (ImageView) findViewById(R.id.wedge_150_degrees);
        orientationWidget[11] = (ImageView) findViewById(R.id.wedge_165_degrees);
        orientationWidget[12] = (ImageView) findViewById(R.id.wedge_180_degrees);
        orientationWidget[13] = (ImageView) findViewById(R.id.wedge_195_degrees);
        orientationWidget[14] = (ImageView) findViewById(R.id.wedge_210_degrees);
        orientationWidget[15] = (ImageView) findViewById(R.id.wedge_225_degrees);
        orientationWidget[16] = (ImageView) findViewById(R.id.wedge_240_degrees);
        orientationWidget[17] = (ImageView) findViewById(R.id.wedge_255_degrees);
        orientationWidget[18] = (ImageView) findViewById(R.id.wedge_270_degrees);
        orientationWidget[19] = (ImageView) findViewById(R.id.wedge_285_degrees);
        orientationWidget[20] = (ImageView) findViewById(R.id.wedge_300_degrees);
        orientationWidget[21] = (ImageView) findViewById(R.id.wedge_315_degrees);
        orientationWidget[22] = (ImageView) findViewById(R.id.wedge_330_degrees);
        orientationWidget[23] = (ImageView) findViewById(R.id.wedge_345_degrees);

        // Initially hide all the widgets.
        int index=0;
        for (ImageView tack : orientationWidget) {
            if( tack!=null) {
                tack.setAlpha(0.0f);
                tack.setVisibility(INVISIBLE);
            }
            else {
                Log.i(CLSS,String.format("initCompass: Wedge %d is null",index));
            }
            index = index+1;
        }
        // Hide the intensity circle.
        animateIntensityCircle(0.0f);
        animateOrientationWidgets(0.0);
        for (ImageView tack : orientationWidget) {
            tack.setVisibility(INVISIBLE);
        }
    }

    /**
     * Initialize the fields with values that can only be determined once the
     * layout for the views has been determined.
     */
    @Override
    protected void onLayout(boolean changed, int l, int t, int r, int b) {
        // Call the parent's onLayout to setup the views.
        super.onLayout(changed, l, t, r, b);

        // The parent container must be a square. A square container simplifies the
        // code. A non-square container does not provide any benefit over a
        // square. Size is in dp. With our current layout squareness does not hold in landscape mode.
        if (mainLayout.getWidth() != mainLayout.getHeight()) {
            Log.i(CLSS, String.format("On Layout NO SQUARE: %d vs %d", mainLayout.getWidth(), mainLayout.getHeight()));
            // throw new IllegalArgumentException(String.format("On Layout NO SQUARE: %d vs %d",mainLayout.getWidth(),mainLayout.getHeight()));
        }
        parentSize = mainLayout.getWidth();
        if (parentSize < 200 || parentSize > 2000) {
            Log.i(CLSS, String.format("On Layout TOO BIG or SMALL: %f", parentSize));
            throw new IllegalArgumentException(String.format("On Layout TOO BIG or SMALL: %f", parentSize));
        }
        // Calculate the center coordinates (radius) of parent container (mainLayout).
        wedgeRadius = mainLayout.getWidth() / 2;
        // Determine the font size for the text view showing linear velocity. 8.3% of the overall size seems to work well.
        magnitudeText.setTextSize(parentSize / 12);
    }

    /**
     * We've received an audio locator message. Annotate the appropriate wedge.
     */
    public void onAudioMessage(AudioLocation msg) {
        boolean forward = (msg.getIntensityFront()>msg.getIntensityBack());
        float magnitude = (float)msg.getIntensityBack();
        if( forward ) magnitude = (float)msg.getIntensityFront();
        float phase = (float)(msg.getPhase()*360./(2.*Math.PI));   // In degrees
        if( !forward ) phase = phase+180.f;
        // Update the size and location (scale and rotation) of various elements.
        animateIntensityCircle(magnitude);
        animateOrientationWidgets(phase);
        updateMagnitudeText(magnitude,phase);
        // Restore the orientation tacks.
        for (ImageView tack : orientationWidget) {
            tack.setVisibility(VISIBLE);
        }

    }

    // ==========================================================================================================

    /**
     * Scale and rotate the intensity circle over the specified duration. Unlike
     * {@link #animateIntensityCircle(float)} this method registers an animation
     * listener.
     *
     * @param endScale The radius that must be attained at the end of the amimation
     */
    private void animateIntensityCircle(float endScale) {
        long duration = FADE_TO_BLACK_DELAY;
        AnimationSet intensityCircleAnimation = new AnimationSet(true);
        intensityCircleAnimation.setInterpolator(new LinearInterpolator());
        intensityCircleAnimation.setFillAfter(true);
        // The listener is needed to set the magnitude text to 0 only after the
        // animation is over.
        intensityCircleAnimation.setAnimationListener(this);
        RotateAnimation rotateAnim;
        rotateAnim = new RotateAnimation(0, 0, wedgeRadius, wedgeRadius);
        rotateAnim.setInterpolator(new LinearInterpolator());
        rotateAnim.setDuration(duration);
        rotateAnim.setFillAfter(true);
        intensityCircleAnimation.addAnimation(rotateAnim);
        ScaleAnimation scaleAnim;
        scaleAnim =
                new ScaleAnimation(0, endScale, 0, endScale, wedgeRadius,
                        wedgeRadius);
        scaleAnim.setDuration(duration);
        scaleAnim.setFillAfter(true);
        intensityCircleAnimation.addAnimation(scaleAnim);
        // Apply the animation.
        intensity.startAnimation(intensityCircleAnimation);
    }

    /**
     * Fade in and fade out the {@link #orientationWidget}s. The widget best
     * aligned with the phase will be the brightest and the
     * successive ones within {@link #ORIENTATION_TACK_FADE_RANGE} the will be
     * faded out proportionally. The tacks out of that range will have alpha set
     * to 0.
     * @param phase  direction in degrees
     */
    private void animateOrientationWidgets(double phase) {
        float deltaTheta;
        for (int i = 0; i < orientationWidget.length; i++) {
            deltaTheta = differenceBetweenAngles(i * 15, (float)phase);
            if (deltaTheta < ORIENTATION_TACK_FADE_RANGE) {
                orientationWidget[i].setAlpha(1.0f - deltaTheta / ORIENTATION_TACK_FADE_RANGE);
            } else {
                orientationWidget[i].setAlpha(0.0f);
            }
        }
    }

    /**
     * From http://actionsnippet.com/?p=1451. Calculates the difference between 2
     * angles. The result is always the minimum difference between 2 angles (0<
     * result <= 360).
     *
     * @param angle0 One of 2 angles used to calculate difference. The order of
     *               arguments does not matter. Must be in degrees.
     * @param angle1 One of 2 angles used to calculate difference. The order of
     *               arguments does not matter. Must be in degrees.
     * @return The difference between the 2 arguments in degrees.
     */
    private float differenceBetweenAngles(float angle0, float angle1) {
        return Math.abs((angle0 + 180 - angle1) % 360 - 180);
    }


    /**
     * Update the linear velocity text view.
     */
    private void updateMagnitudeText(double magnitude,double phase) {
        float normalizedMagnitude = 0f;
        magnitudeText.setText(String.valueOf((int) (normalizedMagnitude * 100)) + "%");
        magnitudeText.setTranslationX((float) (parentSize / 4 * Math.cos((90 + phase)
                    * Math.PI / 180.0)));
        magnitudeText.setTranslationY((float) (parentSize / 4 * Math.sin((90 + phase)
                    * Math.PI / 180.0)));
    }


    /**
     * Comparing 2 float values.
     *
     * @param v1
     * @param v2
     * @return True if v1 and v2 and within {@value #FLOAT_EPSILON} of each other.
     * False otherwise.
     */
    private boolean floatCompare(float v1, float v2) {
        if (Math.abs(v1 - v2) < FLOAT_EPSILON) {
            return true;
        } else {
            return false;
        }
    }

    // ===================================================== Animation Listener ===========================================================
    @Override
    public void onAnimationStart(Animation animation) {
    }

    @Override
    public void onAnimationEnd(Animation animation) {
        float normalizedMagnitude = 0f;
        updateMagnitudeText(normalizedMagnitude,0.0f);
    }

    @Override
    public void onAnimationRepeat(Animation animation) {
    }
}
