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
	private static final int LEFT_JOYSTICK_PORT = 2;
	private static final int MECH_CONTROLLER_PORT = 1;
	public static final int DRIVER_CONTROLLER_PORT = 0;

	private static final int RETRACT_CLIMBER_BUTTON = 3;
	private static final int EXTEND_CLIMBER_BUTTON = 4;

	private static final int INTAKE_BUTTON = 2;
	private static final int OUTTAKE_BUTTON = 1;

	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick leftJoystick;
	private PS4Controller mechController;
	private PS4Controller driverController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
		mechController = new PS4Controller(MECH_CONTROLLER_PORT);
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

	/* ------------------------ Mech Controller ------------------------ */
	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	public double getMechControllerLeftX() {
		return mechController.getLeftX();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getMechControllerLeftY() {
		return mechController.getLeftY();
	}

	/**
	 * Get X axis of Right Joystick.
	 * @return Axis Value
	 */
	public double getMechControllerRightX() {
		return mechController.getRightX();
	}

	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis Value
	 */
	public double getMechControllerRightY() {
		return mechController.getRightY();
	}

	/**
	 * Get the value of the Circle button.
	 * @return True if button is pressed
	 */
	public boolean isMechCircleButtonPressed() {
		return mechController.getCircleButton();
	}

	/**
	 * Get the value of the Triangle button.
	 * @return True if button is pressed
	 */
	public boolean isMechTriangleButtonPressed() {
		return mechController.getTriangleButton();
	}

	/**
	 * Get the value of the Square button.
	 * @return True if button is pressed
	 */
	public boolean isMechSquareButtonPressed() {
		return mechController.getSquareButton();
	}

	/**
	 * Get the value of the Cross button.
	 * @return True if button is pressed
	 */
	public boolean isMechCrossButtonPressed() {
		return mechController.getCrossButton();
	}

	/**
	 * Get the value of the R1 button.
	 * @return True if button is pressed
	 */
	public boolean isMechR1ButtonPressed() {
		return mechController.getR1Button();
	}

	/**
	 * Get the value of the L1 button.
	 * @return True if button is pressed
	 */
	public boolean isMechL1ButtonPressed() {
		return mechController.getL1Button();
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
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return leftJoystick.getRawButton(INTAKE_BUTTON);
	}

	/**
	 * Get the value of the outtake button.
	 * @return True if button is pressed
	 */
	public boolean isOuttakeButtonPressed() {
		return leftJoystick.getRawButton(OUTTAKE_BUTTON);
	}

	/**
	 * Get the value of the retract button.
	 * @return True if button is pressed
	 */
	public boolean isRetractClimberButtonPressed() {
		return leftJoystick.getRawButton(RETRACT_CLIMBER_BUTTON);
	}

	/**
	 * Get the value of the extend button.
	 * @return True if button is pressed
	 */
	public boolean isExtendClimberButtonPressed() {
		return leftJoystick.getRawButton(EXTEND_CLIMBER_BUTTON);

	}
	/* ======================== Private methods ======================== */

}
