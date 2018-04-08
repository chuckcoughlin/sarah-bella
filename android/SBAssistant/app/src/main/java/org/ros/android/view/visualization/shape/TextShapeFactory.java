package org.ros.android.view.visualization.shape;

import android.content.res.AssetManager;
import android.graphics.Typeface;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.text.GLText;

import javax.microedition.khronos.opengles.GL10;


public class TextShapeFactory {
  private final AssetManager assets;
  private final GLText glText;

  public TextShapeFactory(final VisualizationView view, final GL10 gl) {
    assets =  view.getContext().getAssets();
    glText = new GLText(gl,assets);
  }

  public void loadFont(final Typeface typeface, final int size, final int padX, final int padY) {
    glText.load(typeface, size, padX, padY);
  }

  public void loadFont(final String file, final int size, final int padX, final int padY) {
    Typeface tf = Typeface.createFromAsset(assets, file);  // Create the Typeface from Font File
    glText.load(tf, size, padX, padY);
  }

  public TextShape newTextShape(final String text) {
    return new TextShape(glText, text);
  }
}
