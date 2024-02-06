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
	private static final int MECH_CONTROLLER_PORT = 0;
	private static final int INTAKE_BUTTON = 2;
	private static final int OUTTAKE_BUTTON = 1;
	private static final int SHOOT_BUTTON = 3;
	private static final int GROUND_ARM_BUTTON = 4;
	private static final int AMP_ARM_BUTTON = 5;
	private static final int SOURCE_ARM_BUTTON = 6;
	private static final int SHOOTER_ARM_BUTTON = 7;
	private static final int ABORT_BUTTON = 8;

	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick leftJoystick;
	private PS4Controller mechController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
		mechController = new PS4Controller(MECH_CONTROLLER_PORT);
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
	public boolean isGroundArmButtonPressed() {
		return leftJoystick.getRawButton(GROUND_ARM_BUTTON);
	}


	/**
	 * Get the value of the Outtake button.
	 * @return True if button is pressed
	 */
	public boolean isSourceArmButtonPressed() {
		return leftJoystick.getRawButton(SOURCE_ARM_BUTTON);
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
	public boolean isAmpArmButtonPressed() {
		return leftJoystick.getRawButton(AMP_ARM_BUTTON);
	}

	/**
	 * Get the value of the Outtake button.
	 * @return True if button is pressed
	 */
	public boolean isShooterArmButtonPressed() {
		return leftJoystick.getRawButton(SHOOTER_ARM_BUTTON);
	}

	/* ------------------------ Mech Controller ------------------------ */
	/**
	 * Get Y axis of Left Joystick of the controller.
	 * Manual State for PivotFSM
	 * @return Axis value
	 */
	public double getMechLeftJoystickY() {
		return mechController.getLeftY();
	}

	/**
	 * Get Y axis of Right Joystick of the controller.
	 * Possibly to control chain Mech
	 * @return Axis value
	 */
	public double getMecRightJoystickY() {
		return mechController.getRightY();
	}

	/**
	 * Get the value of the Circle Button.
	 * For Shooter State
	 * on the PivotFSM
	 * @return if Circle Button is pressed
	 */
	public boolean isMechCirclePressed() {
		return mechController.getCircleButton();
	}

	/**
	 * Get the value of the Square Button.
	 * For AMP State
	 * on the PivotFSM
	 * @return if Square Button is pressed
	 */
	public boolean isMechSquarePressed() {
		return mechController.getSquareButton();
	}

	/**
	 * Get the value of the Triangle Button.
	 * For Source State
	 * on the PivotFSM
	 * @return if Triangle Button is pressed
	 */
	public boolean isMechTrianglePressed() {
		return mechController.getTriangleButton();
	}

	/**
	 * Get the value of the Cross Button.
	 * For Ground State
	 * on the PivotFSM
	 * @return if Cross Button is pressed
	 */
	public boolean isMechCrossPressed() {
		return mechController.getCrossButton();
	}

	/**
	 * Get the value of the L1 Button.
	 * For Intaking State
	 * on the IntakeFSM
	 * @return if L1 Button is pressed
	 */
	public boolean isLeftBumperPressed() {
		return mechController.getL1Button();
	}

	/**
	 * Get the value of the R1 Button.
	 * For Outtaking State
	 * on the IntakeFSM
	 * @return if R1 Button is pressed
	 */
	public boolean isRightBumperPressed() {
		return mechController.getR1Button();
	}

	/**
	 * Get the value of the Touchpad Button.
	 * For toggle betwen Shooting and Idle States
	 * on the ShooterFSM
	 * @return if Touchpad Button is pressed
	 */
	public boolean isTouchpadPressed() {
		return mechController.getTouchpad();
	}

	/**
	 * Get the value of the R1 Button.
	 * The abort button to stop PIDing
	 * on the PivotFSM
	 * @return if PS Button is pressed
	 */
	public boolean isPSButtonPressed() {
		return mechController.getPSButton();
	}
	/* ======================== Private methods ======================== */

}
