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
	private static final int DRIVER_CONTROLLER_PORT = 0;

	/* ======================== Private variables ======================== */
	// Input objects
	private PS4Controller mechController;
	private PS4Controller driverController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		mechController = new PS4Controller(MECH_CONTROLLER_PORT);
		driverController = new PS4Controller(DRIVER_CONTROLLER_PORT);

	}

	/* ------------------------ Driver Controller ------------------------ */
	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	public double getControllerLeftJoystickY() {
		return driverController.getLeftY();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getControllerLeftJoystickX() {
		return driverController.getLeftX();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getControllerRightJoystickY() {
		return driverController.getRightY();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getControllerRightJoystickX() {
		return driverController.getRightX();
	}
	/**
	 * Get the value of the Share button.
	 * @return True if button is pressed
	 */
	public boolean isBackButtonPressed() {
		return driverController.getShareButton();
	}
	/**
	 * Get the value of the Circle button.
	 * @return True if button is pressed
	 */
	public boolean isCircleButtonPressed() {
		return driverController.getCircleButtonPressed();
	}
	/**
	 * Get the value of the Circle button.
	 * @return True if button is released
	 */
	public boolean isCircleButtonReleased() {
		return driverController.getCircleButtonReleased();
	}
	/**
	 * Get the value of the Triangle button.
	 * @return True if button is pressed
	 */
	public boolean isTriangleButtonPressed() {
		return driverController.getTriangleButtonPressed();
	}
	/**
	 * Get the value of the Circle button.
	 * @return True if button is released
	 */
	public boolean isTriangleButtonReleased() {
		return driverController.getTriangleButtonReleased();
	}
	/**
	 * Get the value of the left trigger.
	 * @return value of the left trigger.
	 */
	public double getLeftTrigger() {
		return driverController.getL2Axis();
	}
	/**
	 * Get the value of the right trigger.
	 * @return value of the right trigger.
	 */
	public double getRightTrigger() {
		return driverController.getR2Axis();
	}

	/* ------------------------ Mech Controller ------------------------ */
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

	/**
	 * Get the value of the Square Button.
	 * @return if Square Button is pressed
	 */
	public boolean isAmpButtonPressed() {
		return mechController.getSquareButton();
	}

	/**
	 * Get the value of the Share Button.
	 * @return if Square Share is pressed
	 */
	public boolean isShootAmpButtonPressed() {
		return mechController.getShareButton();
	}

	/**
	 * Get the value of the L2 Button.
	 * @return if L2 Button is pressed
	 */
	public boolean isoverrideIntakeButtonPressed() {
		return mechController.getL2Button();
	}

	/**
	 * Get the value of the R2 Button.
	 * @return if R2 Button is pressed
	 */
	public boolean isoverrideOuttakeButtonPressed() {
		return mechController.getR2Button();
	}
}
