package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.PS4Controller;
/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int MECH_CONTROLLER_PORT = 1;

	/* ======================== Private variables ======================== */
	// Input objects
	private PS4Controller mechController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		mechController = new PS4Controller(MECH_CONTROLLER_PORT);

	}

	/* ------------------------ Driver Controller ------------------------ */

	/**
	 * Get the value of the Circle Button.
	 * @return if Circle Button is pressed
	 */
	public boolean isShootButtonPressed() {
		return mechController.getTriangleButton();
	}

	/**
	 * Get the value of the R1 Button.
	 * @return if R1 Button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return mechController.getCircleButton();
	}

	/**
	 * Get the value of the Touchpad Button.
	 * @return if Touchpad Button is pressed
	 */
	public boolean isRevButtonPressed() {
		return mechController.getL1Button();
	}


}
