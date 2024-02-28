package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED {
	private AddressableLED led;
	private AddressableLEDBuffer ledBuffer;
	private int rainbowFirstPixelHue = 0;
	private int greenVal = 0;
	private boolean forward = true;

	/**
	 * Constructs LED object.
	 */
	public LED() {
		led = new AddressableLED(9);
		ledBuffer = new AddressableLEDBuffer(30);
		led.setLength(ledBuffer.getLength());
		led.start();
	}

	/**
	 * Changes the LED color to green.
	 */
	public void greenLight() {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, 135, 255, 8);
		}
		led.setData(ledBuffer);
	}

	/**
	 * Changes the LED color to orange.
	 */
	public void orangeLight() {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, 255, 92, 5);
		}
		led.setData(ledBuffer);
	}

	/**
	 * Turns off the LEDs.
	 */
	public void turnOff() {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, 0, 0, 0);
		}
		led.setData(ledBuffer);
	}

	public void cr() {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, 199, greenVal, 0);
		}

		if (greenVal >= 100) {
			forward = false;
		} else if (greenVal <= 0) {
			forward = true;
		}

		if (forward) {
			greenVal++;
		} else {
			greenVal--;
		}

		led.setData(ledBuffer);
		// // For every pixel
		// for (var i = 0; i < ledBuffer.getLength(); i++) {
		//   // Calculate the hue - hue is easier for rainbows because the color
		//   // shape is a circle so only one value needs to precess
		//   final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
		//   // Set the value
		//   ledBuffer.setHSV(i, hue, 255, 128);
		// }
		// // Increase by to make the red to yellow
		// System.out.println("hue value: " + m_rainbowFirstPixelHue);
		// if (m_rainbowFirstPixelHue > 53) {
		//   m_rainbowFirstPixelHue = 0;
		// } else {
		//   m_rainbowFirstPixelHue += 3;
		// }
		// // Check bounds
		// m_rainbowFirstPixelHue %= 180;
		// led.setData(ledBuffer);
	}

	public void rainbow() {
		// For every pixel
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			// Calculate the hue - hue is easier for rainbows because the color
			// shape is a circle so only one value needs to precess
			final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
			// Set the value
			ledBuffer.setHSV(i, hue, 255, 128);
		}
		// Increase by to make the rainbow "move"
		rainbowFirstPixelHue += 3;

		// Check bounds
		rainbowFirstPixelHue %= 180;
		led.setData(ledBuffer);
	}

	public void setRGBVals(int r, int g, int b) {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			// Sets the specified LED to the RGB values for red
			ledBuffer.setRGB(i, r, g, b);
		}
		led.setData(ledBuffer);
	}

	public void setHSVVals(int hue, int saturation, int val) {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			// Sets the specified LED to the HSV values for red
			ledBuffer.setHSV(i, hue, saturation, val);
		}

		led.setData(ledBuffer);
	}

}
