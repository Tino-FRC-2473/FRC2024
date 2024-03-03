package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

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
	private XboxController altMechController;
	private XboxController altDriveController;
	private boolean usingAltDrive = false;
	private boolean usingAltMech = false;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		mechController = new PS4Controller(MECH_CONTROLLER_PORT);
		driverController = new PS4Controller(DRIVER_CONTROLLER_PORT);
		altMechController = new XboxController(MECH_CONTROLLER_PORT);
		altDriveController = new XboxController(DRIVER_CONTROLLER_PORT);
	}

	public void setControllerTypes(boolean usingDriveAlternate, boolean usingMechAlternate) {
		usingAltDrive = usingDriveAlternate;
		usingAltMech = usingMechAlternate;
	}

	/* ------------------------ Driver Controller ------------------------ */
	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	public double getControllerLeftJoystickY() {
		if (!usingAltDrive) {
			return driverController.getLeftY();
		} else {
			return altDriveController.getLeftY();
		}
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getControllerLeftJoystickX() {
		if (!usingAltDrive) {
			return driverController.getLeftX();
		} else {
			return altDriveController.getLeftX();
		}
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getControllerRightJoystickY() {
		if (!usingAltDrive) {
			return driverController.getRightY();
		} else {
			return altDriveController.getRightY();
		}
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getControllerRightJoystickX() {
		if (!usingAltDrive) {
			return driverController.getRightX();
		} else {
			return altDriveController.getRightX();
		}
	}
	/**
	 * Get the value of the Share button.
	 * @return True if button is pressed
	 */
	public boolean isBackButtonPressed() {
		if (!usingAltDrive) {
			return driverController.getShareButton();
		} else {
			return altDriveController.getBackButton();
		}
	}
	/**
	 * Get the value of the Circle button.
	 * @return True if button is pressed
	 */
	public boolean isCircleButtonPressed() {
		if (!usingAltDrive) {
			return driverController.getCircleButtonPressed();
		} else {
			return altDriveController.getBButtonPressed();
		}
	}
	/**
	 * Get the value of the Circle button.
	 * @return True if button is released
	 */
	public boolean isCircleButtonReleased() {
		if (!usingAltDrive) {
			return driverController.getCircleButtonReleased();
		} else {
			return altDriveController.getBButtonReleased();
		}
	}
	/**
	 * Get the value of the Triangle button.
	 * @return True if button is pressed
	 */
	public boolean isTriangleButtonPressed() {
		if (!usingAltDrive) {
			return driverController.getTriangleButtonPressed();
		} else {
			return altDriveController.getYButtonPressed();
		}
	}
	/**
	 * Get the value of the Circle button.
	 * @return True if button is released
	 */
	public boolean isTriangleButtonReleased() {
		if (!usingAltDrive) {
			return driverController.getTriangleButtonReleased();
		} else {
			return altDriveController.getYButtonReleased();
		}
	}
	/**
	 * Get the value of the left trigger.
	 * @return value of the left trigger.
	 */
	public double getLeftTrigger() {
		if (!usingAltDrive) {
			return driverController.getL2Axis();
		} else {
			return altDriveController.getLeftTriggerAxis();
		}
	}
	/**
	 * Get the value of the right trigger.
	 * @return value of the right trigger.
	 */
	public double getRightTrigger() {
		if (!usingAltDrive) {
			return driverController.getR2Axis();
		} else {
			return altDriveController.getRightTriggerAxis();
		}	}

	/* ------------------------ Mech Controller ------------------------ */
	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		if (!usingAltMech) {
			return mechController.getCircleButton();
		} else {
			return altMechController.getBButton();
		}
	}

	/**
	 * Get the value of the shoot button.
	 * @return True if button is pressed
	 */
	public boolean isShootButtonPressed() {
		if (!usingAltMech) {
			return mechController.getTriangleButton();
		} else {
			return altMechController.getYButton();
		}
	}

	/**
	 * Get the value of the rev button for the shooter.
	 * @return True if button is pressed
	 */
	public boolean isRevOuttakeButtonPressed() {
		if (!usingAltMech) {
			return mechController.getL1Button();
		} else {
			return altMechController.getLeftBumper();
		}
	}

	/**
	 * Get the value of the left climber trigger.
	 * @return True if button is pressed
	 */
	public double leftClimberTrigger() {
		if (!usingAltMech) {
			return mechController.getL2Axis();
		} else {
			return altMechController.getLeftTriggerAxis();
		}
	}

	/**
	 * Get the value of the right climber trigger.
	 * @return True if button is pressed
	 */
	public double rightClimberTrigger() {
		if (!usingAltMech) {
			return mechController.getR2Axis();
		} else {
			return altMechController.getRightTriggerAxis();
		}
	}

	/**
	 * Get the value of the toggle cam button.
	 * @return True if button is pressed
	 */
	public boolean chainChamToggleButton() {
		if (!usingAltDrive) {
			return driverController.getR1ButtonPressed();
		} else {
			return altDriveController.getRightBumper();
		}
	}

	/**
	 * Get the value of the synched climber button.
	 * @return True if button is pressed
	 */
	public boolean synchClimberTrigger() {
		if (!usingAltMech) {
			return mechController.getCrossButton();
		} else {
			return altMechController.getAButton();
		}
	}
	public boolean overrideIntakeButton() {
		if (!usingAltMech) {
			return mechController.getSquareButton();
		} else {
			return altMechController.getXButton();
		}
	}
}
