package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_SPARK_SHOOTER_UPPER = 33;
	public static final int CAN_ID_SPARK_SHOOTER_LOWER = 34;

	public static final int CAN_ID_SPARK_LEFT_CLIMBER_MOTOR = 19;
	public static final int CAN_ID_SPARK_RIGHT_CLIMBER_MOTOR = 20;

	// OLD Chassis

	public static final int FRONT_LEFT_DRIVING_CAN_ID = 6;
	public static final int FRONT_RIGHT_DRIVING_CAN_ID = 4;
	public static final int REAR_LEFT_DRIVING_CAN_ID = 8;
	public static final int REAR_RIGHT_DRIVING_CAN_ID = 2;

	public static final int FRONT_LEFT_TURNING_CAN_ID = 5;
	public static final int FRONT_RIGHT_TURNING_CAN_ID = 3;
	public static final int REAR_LEFT_TURNING_CAN_ID = 7;
	public static final int REAR_RIGHT_TURNING_CAN_ID = 1;

	// NEW Chassis

	// public static final int FRONT_LEFT_DRIVING_CAN_ID = 6;
	// public static final int FRONT_RIGHT_DRIVING_CAN_ID = 4;
	// public static final int REAR_LEFT_DRIVING_CAN_ID = 8;
	// public static final int REAR_RIGHT_DRIVING_CAN_ID = 2;

	// public static final int FRONT_LEFT_TURNING_CAN_ID = 5;
	// public static final int FRONT_RIGHT_TURNING_CAN_ID = 3;
	// public static final int REAR_LEFT_TURNING_CAN_ID = 7;
	// public static final int REAR_RIGHT_TURNING_CAN_ID = 1;

	// Pneumatics channel numbers
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_FORWARD = 1;
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_REVERSE = 2;

	// Place jumper from DIO pin 9 to GND to indicate this is a test setup
	private static final int DIO_TEST_SETUP_CHANNEL = 9;
	private static DigitalInput testBoardPin = new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL);
	/**
	 * Check if the current RoboRIO is part of a test setup or real robot.
	 * @return true if the current setup is a test setup
	 */
	public static boolean isTestBoard() {
		return !HardwareMap.testBoardPin.get();
	}
}
