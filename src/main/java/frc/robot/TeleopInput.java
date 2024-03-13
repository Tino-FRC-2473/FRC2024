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

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Mech Controller ------------------------ */
	/**
	 * Get Y axis of Left Joystick of the controller.
	 * Manual Control for PivotFSM
	 * @return Axis value
	 */

	public double getMechControllerLeftY() {
		return mechController.getLeftY();
	}

	/**
	 * Get the value of the Circle Button.
	 * @return if Circle Button is pressed
	 */
	public boolean isShooterArmButtonPressed() {
		return mechController.getCircleButton();
	}

	/**
	 * Get the value of the Square Button.
	 * @return if Square Button is pressed
	 */
	public boolean isAmpArmButtonPressed() {
		return mechController.getSquareButton();
	}

	/**
	 * Get the value of the Triangle Button.
	 * @return if Triangle Button is pressed
	 */
	public boolean isSourceArmButtonPressed() {
		return mechController.getTriangleButton();
	}

	/**
	 * Get the value of the Cross Button.
	 * @return if Cross Button is pressed
	 */
	public boolean isGroundArmButtonPressed() {
		return mechController.getCrossButton();
	}

	/**
	 * Get the value of the L1 Button.
	 * @return if L1 Button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return mechController.getL1Button();
	}

	/**
	 * Get the value of the R1 Button.
	 * @return if R1 Button is pressed
	 */
	public boolean isOuttakeButtonPressed() {
		return mechController.getR1Button();
	}

	/**
	 * Get the value of the Touchpad Button.
	 * @return if Touchpad Button is pressed
	 */
	public boolean isShootButtonPressed() {
		return mechController.getTouchpad();
	}

	/**
	 * Get the value of the PS Button.
	 * @return if PS Button is pressed
	 */
	public boolean isAbortButtonPressed() {
		return mechController.getPSButton();
	}

	/**
	 * Get the Value of the Options Button.
	 * @return if Options Button is pressed
	 */
	public boolean isZeroingButtonPressed() {
		return mechController.getOptionsButton();
	}
	/* ======================== Private methods ======================== */

}
