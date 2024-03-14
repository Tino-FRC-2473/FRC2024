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
	 * Get Y axis of Left Joystick of the controller.
	 * Manual Control for PivotFSM
	 * @return Axis value
	 */

	public double getMechControllerLeftY() {
		return mechController.getLeftY();
	}

	public double getControllerLeftJoystickY() {
		return driverController.getLeftY();
	}

	/**
	 * Get the value of the Circle Button.
	 * @return if Circle Button is pressed
	 */
	public boolean isShooterArmButtonPressed() {
		return mechController.getCircleButton();
	}
	
	public double getControllerLeftJoystickX() {
		return driverController.getLeftX();
	}


	public boolean isAmpArmButtonPressed() {
		return mechController.getSquareButton();
	}

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

}
