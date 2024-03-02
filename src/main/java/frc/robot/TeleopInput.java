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
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return mechController.getCircleButton();
	}

	/**
	 * Get the value of the shoot button.
	 * @return True if button is pressed
	 */
	public boolean isShootButtonPressed() {
		return mechController.getTriangleButton();
	}

	/**
	 * Get the value of the rev button for the shooter.
	 * @return True if button is pressed
	 */
	public boolean isRevOuttakeButtonPressed() {
		return mechController.getL1Button();
	}

	/**
	 * Get the value of the left climber trigger.
	 * @return True if button is pressed
	 */
	public double leftClimberTrigger() {
		return mechController.getL2Axis();
	}

	/**
	 * Get the value of the right climber trigger.
	 * @return True if button is pressed
	 */
	public double rightClimberTrigger() {
		return mechController.getR2Axis();
	}

	/**
	 * Get the value of the toggle cam button.
	 * @return True if button is pressed
	 */
	public boolean chainChamToggleButton() {
		return driverController.getR1ButtonPressed();
	}

	/**
	 * Get the value of the synched climber button.
	 * @return True if button is pressed
	 */
	public boolean synchClimberTrigger() {
		return mechController.getCrossButton();
	}
	public boolean overrideIntakeButton() {
		return mechController.getSquareButton();
	}
}
