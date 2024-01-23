package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int LEFT_JOYSTICK_PORT = 0;
	private static final int PS5_CONTROLLER_PORT = 1;
	private static final int INTAKE_BUTTON = 2;
	private static final int OUTTAKE_BUTTON = 1;
	private static final int SHOOT_BUTTON = 3;
	private static final int GROUND_POS_BUTTON = 4;
	private static final int AMP_POS_BUTTON = 5;
	private static final int SOURCE_POS_BUTTON = 6;
	private static final int SHOOTER_POS_BUTTON = 7;
	private static final int ABORT_BUTTON = 8;

	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick leftJoystick;
	private PS5Controller controller;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
		controller = new PS5Controller(PS5_CONTROLLER_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

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
	public boolean isShootButtonPressed() {
		return leftJoystick.getRawButton(SHOOT_BUTTON);
	}
	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return leftJoystick.getRawButton(INTAKE_BUTTON);
	}

	/**
	 * Get the value of the Outtake button.
	 * @return True if button is pressed
	 */
	public boolean isOuttakeButtonPressed() {
		return leftJoystick.getRawButton(OUTTAKE_BUTTON);
	}

	/**
	 * Get the value of the Outtake button.
	 * @return True if button is pressed
	 */
	public boolean isGroundButtonPressed() {
		return leftJoystick.getRawButton(GROUND_POS_BUTTON);
	}


	/**
	 * Get the value of the Outtake button.
	 * @return True if button is pressed
	 */
	public boolean isSourceButtonPressed() {
		return leftJoystick.getRawButton(SOURCE_POS_BUTTON);
	}

	/**
	 * Get the value of the Outtake button.
	 * @return True if button is pressed
	 */
	public boolean isAbortButtonPressed() {
		return leftJoystick.getRawButton(ABORT_BUTTON);
	}

	/**
	 * Get the value of the Outtake button.
	 * @return True if button is pressed
	 */
	public boolean isAmpButtonPressed() {
		return leftJoystick.getRawButton(AMP_POS_BUTTON);
	}

	/**
	 * Get the value of the Outtake button.
	 * @return True if button is pressed
	 */
	public boolean isShooterButtonPressed() {
		return leftJoystick.getRawButton(SHOOTER_POS_BUTTON);
	}

	/* ------------------------ Right Joystick ------------------------ */
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickX() {
		return controller.getR2Axis();
	}
	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickY() {
		return controller.getL2Axis();
	}

	/* ======================== Private methods ======================== */

}
