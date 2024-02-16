package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_SPARK_SHOOTER_UPPER = 31;
	public static final int CAN_ID_SPARK_SHOOTER_LOWER = 30;

	public static final int CAN_ID_SPARK_LEFT_CLIMBER_MOTOR = 5;
	public static final int CAN_ID_SPARK_RIGHT_CLIMBER_MOTOR = 6;

	public static final int FRONT_LEFT_DRIVING_CAN_ID = 8;
	public static final int FRONT_RIGHT_DRIVING_CAN_ID = 6;
	public static final int REAR_LEFT_DRIVING_CAN_ID = 2;
	public static final int REAR_RIGHT_DRIVING_CAN_ID = 4;

	public static final int FRONT_LEFT_TURNING_CAN_ID = 7;
	public static final int FRONT_RIGHT_TURNING_CAN_ID = 5;
	public static final int REAR_LEFT_TURNING_CAN_ID = 1;
	public static final int REAR_RIGHT_TURNING_CAN_ID = 3;
}
