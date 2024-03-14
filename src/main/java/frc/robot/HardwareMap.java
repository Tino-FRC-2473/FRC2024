package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_SPARK_DRIVE_FRONT_RIGHT = 1;
	public static final int CAN_ID_SPARK_DRIVE_BACK_RIGHT = 2;
	public static final int CAN_ID_SPARK_DRIVE_FRONT_LEFT = 3;
	public static final int CAN_ID_SPARK_DRIVE_BACK_LEFT = 4;

	public static final int CAN_ID_SPARK_LSHOOTER_MOTOR = 36;
	public static final int CAN_ID_SPARK_RSHOOTER_MOTOR = 35;
	public static final int CAN_ID_SPARK_INTAKE_MOTOR = 31;
	public static final int DEVICE_ID_ARM_MOTOR = 0;

	public static final int CAN_SPARK_PIVOT_MOTOR = 36;

	public static final int INPUT_LIMIT_SWITCH_PORT = 0;

	// public static final int CAN_ID_SPARK_LEFT_CLIMBER_MOTOR = 19;
	// public static final int CAN_ID_SPARK_RIGHT_CLIMBER_MOTOR = 20;

	// NEW Chassis

	public static final int FRONT_LEFT_DRIVING_CAN_ID = 6;
	public static final int FRONT_RIGHT_DRIVING_CAN_ID = 4;
	public static final int REAR_LEFT_DRIVING_CAN_ID = 8;
	public static final int REAR_RIGHT_DRIVING_CAN_ID = 2;

	public static final int FRONT_LEFT_TURNING_CAN_ID = 5;
	public static final int FRONT_RIGHT_TURNING_CAN_ID = 3;
	public static final int REAR_LEFT_TURNING_CAN_ID = 7;
	public static final int REAR_RIGHT_TURNING_CAN_ID = 1;

	// OLD Chassis

	// public static final int FRONT_LEFT_DRIVING_CAN_ID = 8;
	// public static final int FRONT_RIGHT_DRIVING_CAN_ID = 6;
	// public static final int REAR_LEFT_DRIVING_CAN_ID = 2;
	// public static final int REAR_RIGHT_DRIVING_CAN_ID = 4;

	// public static final int FRONT_LEFT_TURNING_CAN_ID = 7;
	// public static final int FRONT_RIGHT_TURNING_CAN_ID = 5;
	// public static final int REAR_LEFT_TURNING_CAN_ID = 1;
	// public static final int REAR_RIGHT_TURNING_CAN_ID = 3;
}
