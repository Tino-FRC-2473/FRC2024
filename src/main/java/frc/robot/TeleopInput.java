package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;
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
	private static final int LEFT_JOYSTICK_PORT = 1;
	private static final int RIGHT_JOYSTICK_PORT = 2;
	private static final int ELEVATOR_HIGH_BUTTON = 7;
	private static final int ELEVATOR_MID_BUTTON = 9;
	private static final int ELEVATOR_LOW_BUTTON = 11;
	private static final int WRIST_IN_BUTTON = 6;
	private static final int WRIST_OUT_BUTTON = 4;
	private static final int WRIST_IN_BUTTON_DOUBLE = 5;
	private static final int WRIST_OUT_BUTTON_DOUBLE = 3;
	private static final int ABORT_FLIP_BUTTON = 12;
	private static final int INTAKE_BUTTON = 2;
	private static final int FLIP_BUTTON = 10;
	private static final int ARM_ZERO_BUTTON = 8;
	//private static final int WRIST_ZERO_BUTTON = -1;

	public static final int DRIVER_CONTROLLER_PORT = 0;


	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick leftJoystick;
	private Joystick rightJoystick;
	private PS4Controller driverController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
		rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
		driverController = new PS4Controller(DRIVER_CONTROLLER_PORT);

	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

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

	/* ------------------------ Left Joystick ------------------------ */
	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickX() {
		return leftJoystick.getX();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickY() {
		return leftJoystick.getY();
	}
	/**
	 * Get the value of the shooter button.
	 * @return True if button is pressed
	 */
	public boolean isShooterButtonPressed() {
		return leftJoystick.getRawButton(1);
	}

	/**
	 * Get the value of the flip button.
	 * @return True if the button is pressed
	 */
	public boolean isFlipButtonPressed() {
		return leftJoystick.getRawButtonPressed(FLIP_BUTTON);
	}

	/* ------------------------ Right Joystick ------------------------ */
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickX() {
		return rightJoystick.getX();
	}
	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickY() {
		return rightJoystick.getY();
	}

	/**
	 * Get the value of the high button.
	 * @return True if button is pressed
	 */
	public boolean isHighButtonPressed() {
		return leftJoystick.getRawButton(ELEVATOR_HIGH_BUTTON);
	}

	/**
	 * Get the value of the mid button.
	 * @return True if button is pressed
	 */
	public boolean isMidButtonPressed() {
		return leftJoystick.getRawButton(ELEVATOR_MID_BUTTON);
	}

	/**
	 * Get the value of the low button.
	 * @return True if button is pressed
	 */
	public boolean isLowButtonPressed() {
		return leftJoystick.getRawButton(ELEVATOR_LOW_BUTTON);
	}

	/**
	 * Get the value of the release button.
	 * @return True if button is pressed
	 */
	public boolean isOuttakeButtonPressed() {
		return leftJoystick.getTrigger();
	}

	/**
	 * Get the value of the wrist down button.
	 * @return True if button is pressed
	 */
	public boolean isWristOutButtonPressed() {
		return leftJoystick.getRawButton(WRIST_OUT_BUTTON);
	}

	/**
	 * Get the value of the wrist up button.
	 * @return True if button is pressed
	 */
	public boolean isWristInButtonPressed() {
		return leftJoystick.getRawButton(WRIST_IN_BUTTON);
	}

		/**
	 * Get the value of the wrist down button.
	 * @return True if button is pressed
	 */
	public boolean isWristOutDoubleButtonPressed() {
		return leftJoystick.getRawButton(WRIST_OUT_BUTTON_DOUBLE);
	}

	/**
	 * Get the value of the wrist up button.
	 * @return True if button is pressed
	 */
	public boolean isWristInDoubleButtonPressed() {
		return leftJoystick.getRawButton(WRIST_IN_BUTTON_DOUBLE);
	}

	/**
	 * Get the value of the flip abort button.
	 * @return True if button is pressed
	 */
	public boolean isFlipAbortButtonPressed() {
		return leftJoystick.getRawButton(ABORT_FLIP_BUTTON);
	}

	/**
	 * Get the value of the fine tuning button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return leftJoystick.getRawButton(INTAKE_BUTTON);
	}

	/**
	 * Get the value of the arm zero button.
	 * @return True if button is pressed
	 */
	public boolean isArmZeroButtonPressed() {
		return leftJoystick.getRawButton(ARM_ZERO_BUTTON);
	}

	/**
	 * Get the value of the throttle.
	 * @return True throttle is forward
	 */
	public boolean isThrottleForward() {
		return leftJoystick.getThrottle() <= 0;
	}
	/* ======================== Private methods ======================== */

}
