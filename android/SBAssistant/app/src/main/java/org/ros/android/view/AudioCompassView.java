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

import audio_locator.SignalAudio;
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
     * normalizedMagnitude This is the distance between the center divet and the
     * point of contact normalized between 0 and 1. The linear velocity is based
     * on this.
     */
    private float normalizedMagnitude;
    /**
     * normalizingMultiplier Used to convert any distance from pixels to a
     * normalized value between 0 and 1. 0 is the center of widget and 1 is the
     * normalized distance to the {@link #outerBand} from the center of the
     * widget.
     */
    private float normalizingMultiplier;
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
        orientationWidget[1] = (ImageView) findViewById(R.id.widget_15_degrees);
        orientationWidget[2] = (ImageView) findViewById(R.id.widget_30_degrees);
        orientationWidget[3] = (ImageView) findViewById(R.id.widget_45_degrees);
        orientationWidget[4] = (ImageView) findViewById(R.id.widget_60_degrees);
        orientationWidget[5] = (ImageView) findViewById(R.id.widget_75_degrees);
        orientationWidget[6] = (ImageView) findViewById(R.id.widget_90_degrees);
        orientationWidget[7] = (ImageView) findViewById(R.id.widget_105_degrees);
        orientationWidget[8] = (ImageView) findViewById(R.id.widget_120_degrees);
        orientationWidget[9] = (ImageView) findViewById(R.id.widget_135_degrees);
        orientationWidget[10] = (ImageView) findViewById(R.id.widget_150_degrees);
        orientationWidget[11] = (ImageView) findViewById(R.id.widget_165_degrees);
        orientationWidget[12] = (ImageView) findViewById(R.id.widget_180_degrees);
        orientationWidget[13] = (ImageView) findViewById(R.id.widget_195_degrees);
        orientationWidget[14] = (ImageView) findViewById(R.id.widget_210_degrees);
        orientationWidget[15] = (ImageView) findViewById(R.id.widget_225_degrees);
        orientationWidget[16] = (ImageView) findViewById(R.id.widget_240_degrees);
        orientationWidget[17] = (ImageView) findViewById(R.id.widget_255_degrees);
        orientationWidget[18] = (ImageView) findViewById(R.id.widget_270_degrees);
        orientationWidget[19] = (ImageView) findViewById(R.id.widget_285_degrees);
        orientationWidget[20] = (ImageView) findViewById(R.id.widget_300_degrees);
        orientationWidget[21] = (ImageView) findViewById(R.id.widget_315_degrees);
        orientationWidget[22] = (ImageView) findViewById(R.id.widget_330_degrees);
        orientationWidget[23] = (ImageView) findViewById(R.id.widget_345_degrees);

        // Initially hide all the widgets.
        for (ImageView tack : orientationWidget) {
            tack.setAlpha(0.0f);
            tack.setVisibility(INVISIBLE);
        }
        // Hide the intensity circle.
        animateIntensityCircle(0);
        animateOrientationWidgets();
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
    public void onAudioMessage(SignalAudio msg) {
        // Update the size and location (scale and rotation) of various elements.
        animateIntensityCircle();
        animateOrientationWidgets();
        updateMagnitudeText();
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
     * @param endScale The scale factor that must be attained at the end of the
     *                 animation.
     * @param duration The duration in milliseconds the animation should take.
     */
    private void animateIntensityCircle(float endScale, long duration) {
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
     * aligned with the {@link #contactTheta} will be the brightest and the
     * successive ones within {@link #ORIENTATION_TACK_FADE_RANGE} the will be
     * faded out proportionally. The tacks out of that range will have alpha set
     * to 0.
     */
    private void animateOrientationWidgets() {
        float deltaTheta;
        for (int i = 0; i < orientationWidget.length; i++) {
            deltaTheta = differenceBetweenAngles(i * 15, contactTheta);
            if (deltaTheta < ORIENTATION_TACK_FADE_RANGE) {
                orientationWidget[i].setAlpha(1.0f - deltaTheta / ORIENTATION_TACK_FADE_RANGE);
            } else {
                orientationWidget[i].setAlpha(0.0f);
            }
        }
    }


    /**
     * Updates the virtual joystick layout based on the location of the contact.
     * Generates the velocity messages. Switches in and out of turn-in-place.
     *
     * @param x The x coordinates of the contact relative to the parent container.
     * @param y The y coordinates of the contact relative to the parent container.
     */
    private void onContactMove(float x, float y) {
        // Get the coordinates of the contact relative to the center of the main
        // layout.
        float thumbDivetX = x - joystickRadius;
        float thumbDivetY = y - joystickRadius;
        // Convert the coordinates from Cartesian to Polar.
        contactTheta = (float) (Math.atan2(thumbDivetY, thumbDivetX) * 180 / Math.PI + 90);
        contactRadius =
                (float) Math.sqrt(thumbDivetX * thumbDivetX + thumbDivetY * thumbDivetY)
                        * normalizingMultiplier;
        // Calculate the distance (0 to 1) from the center divet to the contact
        // point.
        normalizedMagnitude = (contactRadius - deadZoneRatio) / (1 - deadZoneRatio);
        // Perform bounds checking.
        if (contactRadius >= 1f) {
            // Since the contact is outside the outer ring, reset the coordinate for
            // the thumb divet to the on the outer ring.
            thumbDivetX /= contactRadius;
            thumbDivetY /= contactRadius;
            // The magnitude should not exceed 1.
            normalizedMagnitude = 1f;
            contactRadius = 1f;
        } else if (contactRadius < deadZoneRatio) {
            // Since the contact is inside the dead zone snap the thumb divet to the
            // dead zone. It should stay there till the contact gets outside the
            // deadzone area.
            thumbDivetX = 0;
            thumbDivetY = 0;
            // Prevent normalizedMagnitude going negative inside the deadzone.
            normalizedMagnitude = 0f;
        }

        // Magnetize!
        // If the contact is not snapped to the x axis.
        if (!magnetizedXAxis) {
            // Check if the contact should be snapped to either axis.
            if ((contactTheta + 360) % 90 < magnetTheta) {
                // If the current angle is within MAGNET_THETA degrees + 0, 90, 180, or
                // 270 then subtract the additional degrees so that the current theta is
                // 0, 90, 180, or 270.
                contactTheta -= ((contactTheta + 360) % 90);
            } else if ((contactTheta + 360) % 90 > (90 - magnetTheta)) {
                // If the current angle is within MAGNET_THETA degrees - 0, 90, 180, or
                // 270 then add the additional degrees so that the current theta is 0,
                // 90, 180, or 270.
                contactTheta += (90 - ((contactTheta + 360) % 90));
            }
            // Indicate that the contact has been snapped to the x-axis.
            if (floatCompare(contactTheta, 90) || floatCompare(contactTheta, 270)) {
                magnetizedXAxis = true;
            }
        } else {
            // Use a wider range to keep the contact snapped in.
            if (differenceBetweenAngles((contactTheta + 360) % 360, 90) < POST_LOCK_MAGNET_THETA) {
                contactTheta = 90;
            } else if (differenceBetweenAngles((contactTheta + 360) % 360, 270) < POST_LOCK_MAGNET_THETA) {
                contactTheta = 270;
            }
            // Indicate that the contact is not snapped to the x-axis.
            else {
                magnetizedXAxis = false;
            }
        }

        // Update the size and location (scale and rotation) of various elements.
        animateIntensityCircle(contactRadius);
        animateOrientationWidgets();
        updateThumbDivet(thumbDivetX, thumbDivetY);
        updateMagnitudeText();
        // Command the velocities. There are only two degrees of freedom. We choose velocity in x and rotation.
        // (The original called these holonomic/non-holonomic, but I disagree).
        controller.commandVelocity(normalizedMagnitude * Math.cos(contactTheta * Math.PI / 180.0),
                normalizedMagnitude * Math.sin(contactTheta * Math.PI / 180.0));

        // Check if the turn-in-place mode needs to be activated/deactivated.
        updateTurnInPlaceMode();
    }


    /**
     * Update the linear velocity text view.
     */
    private void updateMagnitudeText() {
        magnitudeText.setText(String.valueOf((int) (normalizedMagnitude * 100)) + "%");
        magnitudeText.setTranslationX((float) (parentSize / 4 * Math.cos((90 + contactTheta)
                    * Math.PI / 180.0)));
        magnitudeText.setTranslationY((float) (parentSize / 4 * Math.sin((90 + contactTheta)
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
        normalizedMagnitude = 0f;
        updateMagnitudeText();
    }

    @Override
    public void onAnimationRepeat(Animation animation) {
    }
}