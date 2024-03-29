package org.ghrobotics.frc2023.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  // LED and LED buffer.
  private final AddressableLED om_;
  private final AddressableLEDBuffer led_buffer_;

  // Keeps track of rainbow pattern.
  int rainbow_first_pixel_hue_ = 0;

  // Keeps track of snake pattern.
  int snake_first_index_ = 0;
  int snake_multiplier_ = 1;

  // IO
  private OutputType output_type_ = OutputType.DISABLED_READY;
  private StandardLEDOutput standard_led_output_;

  public LED() {
    // Initialize LED and LED buffer.
    led_buffer_ = new AddressableLEDBuffer(Constants.kBufferSize);
    om_ = new AddressableLED(Constants.kPortId);
    om_.setLength(led_buffer_.getLength());
    om_.start();

  }

  /**
   * Runs periodically every 20 ms. Here, the LED output is set.
   */
  @Override
  public void periodic() {
    // Write outputs.
    switch (output_type_) {
      case STANDARD:
      //System.out.println("Inside Standard");
        // Calculate the % of time that we remain on.
        double total_cycle_time = standard_led_output_.on_time + standard_led_output_.off_time;
        double on_ratio = standard_led_output_.on_time / total_cycle_time;

        // Calculate which % of the time we are in.
        double remainder = Timer.getFPGATimestamp() % total_cycle_time;
        boolean on = remainder / total_cycle_time <= on_ratio;

        // Set the buffer.
        for (int i = 0; i < Constants.kBufferSize; i++) {
          led_buffer_.setLED(i, on ? standard_led_output_.c : Color.kBlack);
        }
        break;

      case DISABLED_READY:
      //System.out.println("Inside Disabled Ready");
        setRainbow();
        break;

      case ENABLED_READY:
      //System.out.println("Inside Enabled Ready");
        setSnake(Color.kGreen);
        break;

      case TRACKING_TARGET:
        setSnake(Color.kRoyalBlue);
        break;

      /**
       case AUTOBALANCING:
       setSnake(Color.kDeepPink);
       break;

       case ERROR:
       setSnake(Color.kRed);
       break;

       case LIMELIGHT_ERROR:
       setSnake(Color.kOrange);
       break;
       **/
    }


    // Set the LED data.
    om_.setData(led_buffer_);
  }

  /**
   * Sets the standard (solid color for duration of time) LED output.
   *
   * @param output The LED output.
   */
  public void setOutput(StandardLEDOutput output) {
    output_type_ = OutputType.STANDARD;
    standard_led_output_ = output;
  }

  /**
   * Sets the special (non-standard) LED output.
   *
   * @param type The type of special LED output.
   */
  public void setOutput(OutputType type) {
    output_type_ = type;
    standard_led_output_ = StandardLEDOutput.BLANK;
  }

  /**
   * Calculates a moving rainbow and sets the LED buffer.
   */
  private void setRainbow() {
    for (int i = 0; i < Constants.kBufferSize; i++) {
      // Calculate the hue: hue is easier for rainbows because the color shape is a circle, so
      // only one value needs to precess.
      int hue = (rainbow_first_pixel_hue_ + (i * 180 / Constants.kBufferSize)) % 180;

      // Set the value.
       led_buffer_.setHSV(i, hue, 255, 128);
       //System.out.println("hue: "+hue);
      // led_buffer_.setHSV(i, 180, 255, 128);
      //led_buffer_.setRGB(i, 0, 255, 0);
    }

    // Increase by 3 to make the rainbow move.
     rainbow_first_pixel_hue_ += 3;

    // Check bounds.
     rainbow_first_pixel_hue_ %= 180;
  }

  /**
   * Calculates a moving snake pattern and sets the LED buffer.
   *
   * @param color The color to use for the snake pattern.
   */
  private void setSnake(Color color) {
    // Update snake multiplier.
    if (snake_first_index_ == 0)
      snake_multiplier_ = 1;
    if (snake_first_index_ == Constants.kBufferSize - Constants.kSnakeOnLEDs)
      snake_multiplier_ = -1;

    // Set all LEDs to black.
    for (int i = 0; i < Constants.kBufferSize; i++)
      led_buffer_.setRGB(i, 0, 0, 0);

    // Set snake pattern LEDs to color.
    for (int i = 0; i < Constants.kSnakeOnLEDs; i++)
      led_buffer_.setLED(snake_first_index_ + i, color);

    // Increment first index.
    snake_first_index_ += snake_multiplier_;
  }

  public enum OutputType {
    STANDARD, DISABLED_READY, ENABLED_READY, TRACKING_TARGET
    //AUTOBALANCING, ERROR, LIMELIGHT_ERROR
  }

  public enum StandardLEDOutput {
    // Blank
    BLANK(Color.kBlack, 1.0, 0.0),

    // Robot Cases
    // ENABLED_READY(Color.kGreen, 1.0, 0.0),
    AUTOBALANCING(Color.kDeepPink, 1.0, 0.0),
    ERROR(Color.kRed, 1.0, 0.0),
    AUTO(Color.kHotPink, 1.0, 0.0),
    LIMELIGHT_ERROR(Color.kRed, 0.5, 0.5),
    CUBE(Color.kPurple, 1.0, 0.0),
    CONE(Color.kYellow, 1.0, 0.0),
    CUBE_ALIGNING(Color.kPurple, 0.0625, 0.0625),
    CONE_ALIGNING(Color.kYellow, 0.0625, 0.0625);


    // Stores the color and on percentage for the current output.
    Color c;
    double on_time;
    double off_time;

    /**
     * Constructs an enum instance of a standard LED output (solid color with an on-off
     * duration).
     *
     * @param c        The color of the LED.
     * @param on_time  The length of time that the LED is on.
     * @param off_time The length of time that the LED is off.
     */
    StandardLEDOutput(Color c, double on_time, double off_time) {
      this.c = c;
      this.on_time = on_time;
      this.off_time = off_time;
    }
  }

  public static class Constants {
    public static final int kPortId = 9;
    public static final int kBufferSize = 30;

    public static final int kSnakeOnLEDs = 3;
  }
}
