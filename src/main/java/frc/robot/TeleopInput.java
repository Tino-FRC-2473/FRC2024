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

	/* ------------------------ Mech Controller ------------------------ */
	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isRunMotorPressed() {
		return mechController.getCircleButton();
	}

	/**
	 * Get the value of the retract button.
	 * @return True if button is pressed
	 */
	public boolean isPlayMusicPressed() {
		return mechController.getCrossButton();
	}

}
