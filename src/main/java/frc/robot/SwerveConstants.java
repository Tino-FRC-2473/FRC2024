// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class SwerveConstants {
	public static final class DriveConstants {
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double MAX_SPEED_METERS_PER_SECOND = 6; //4.8
		public static final double MAX_ANGULAR_SPEED = 3 * Math.PI; //2 PI // radians per second

		public static final double LEFT_TRIGGER_DRIVE_CONSTANT = 1.5;
		public static final double ANGULAR_SPEED_LIMIT_CONSTANT = 1.5;
		public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
		public static final double MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
		public static final double ROTATIONAL_SLEW_RATE = 2.0; // percent per second (1 = 100%)
		public static final double INSTANTANEOUS_SLEW_RATE = 500;
		//some high number that means the slewrate is effectively instantaneous

		// Chassis configuration
		public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
		// Distance between centers of right and left wheels on robot
		public static final double WHEEL_BASE = Units.inchesToMeters(22.75);
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
			new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
			new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
			new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
			new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

		// Angular offsets of the modules relative to the chassis in radians
		public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
		public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
		public static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
		public static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

		public static final boolean GYRO_REVERSED = false;
		public static final double TIME_CONSTANT = 1e-6;
		public static final double CURRENT_THRESHOLD = 1e-4;
		// some small number to avoid floating-point errors with equality checking
		public static final double ANGLE_MULTIPLIER_1 = 0.45;
		public static final double ANGLE_MULTIPLIER_2 = 0.85;
	}

	public static final class ModuleConstants {
		// The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
		// This changes the drive speed of the module (a pinion gear with more teeth will result in
		// a robot that drives faster).
		public static final int DRIVING_MOTOR_PINON_TEETH = 13;

		// Invert the turning encoder, since the output shaft rotates in the opposite direction of
		// the steering motor in the MAXSwerve Module.
		public static final boolean TURNING_MOTOR_INVERTED = true;

		// Calculations required for driving motor conversion factors and feed forward
		public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.
			FREE_SPEED_RPM / 60;
		public static final double WHEEL_DIAMETER_METERS = 0.0762;
		public static final double WHEEL_CIRCUMFRENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear,
		// 15 teeth on the bevel pinion
		public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22)
			/ (DRIVING_MOTOR_PINON_TEETH * 15);
		public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
			* WHEEL_CIRCUMFRENCE_METERS) / DRIVING_MOTOR_REDUCTION;

		public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS
			* Math.PI) / DRIVING_MOTOR_REDUCTION; // meters
		public static final double DRIVING_ENCODOR_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS
			* Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

		public static final double TURNING_ENCODER_POSITION_FACTOR = (
			2 * Math.PI); // radians
		public static final double TURNING_ENCODER_VELOCITY_FACTOR = (
			2 * Math.PI) / 60.0; // radians per second

		public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
		public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT
				= TURNING_ENCODER_POSITION_FACTOR; // radians

		public static final double DRIVING_P = 0.04;
		public static final double DRIVING_I = 0;
		public static final double DRIVING_D = 0;
		public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
		public static final double DRIVING_MIN_OUTPUT = -1;
		public static final double DRIVING_MAX_OUTPUT = 1;

		public static final double TURNING_P = 1;
		public static final double TURNING_I = 0;
		public static final double TURNING_D = 0;
		public static final double TURNING_FF = 0;
		public static final double TURNING_MIN_OUTPUT = -1;
		public static final double TURNING_MAX_OUTPUT = 1;

		public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
		public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

		public static final int DRIVING_MOTOR_CUTTENT_LIMIT = 40; // amps
		public static final int TURNING_MOTOR_CURRENT_LIMIT = 30; // amps
	}

	public static final class OIConstants {
		public static final double DRIVE_DEADBAND = 0.02;
	}

	public static final class AutoConstants {
		//auto paths constants
		public static final double MAX_SPEED_METERS_PER_SECOND = 0.65; // 0.5 decided
		public static final double MAX_SPEED_METERS_PER_SECOND_FAST = 0.75; // 0.5 decided
		public static final double MAX_ACCEL_METERS_PER_SECOND = 0.7;

		public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI / 10;
		public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI / 2;
		public static final double AUTO_DRIVE_METERS_MARGIN_OF_ERROR = 0.03;
		public static final double AUTO_DRIVE_METERS_MARGIN_OF_ERROR_FAST = 0.05;
		public static final double AUTO_DRIVE_DEGREES_MARGIN_OF_ERROR = 1.5;
		public static final double AUTO_DRIVE_ANGULAR_SPEED_ACCEL_CONSTANT = 70;
		public static final double AUTO_DRIVE_TRANSLATIONAL_SPEED_ACCEL_CONSTANT = 2.5;
		public static final double CONSTANT_SPEED_THRESHOLD = 0.3; // meters
		public static final double WAIT_TIME = 5; // seconds

		// constants for auto path points
		public static final double N_0_05 = 0.05;
		public static final double N_0_10 = 0.1;
		public static final double N_0_25 = 0.25;
		public static final double N_0_5 = 0.5;
		public static final double N_1_5 = 1.5;
		public static final double N_2 = 2;
		public static final double N_2_5 = 2.5;
		public static final double N_3 = 3;
		public static final double N_3_5 = 3.5;
		public static final double N_4 = 4;
		public static final double N_4_5 = 4.5;
		public static final double N_5 = 5;
		public static final double N_5_5 = 5.5;
		public static final double N_6 = 6;
		public static final double N_6_5 = 6.5;
		public static final double N_7 = 7;
		public static final double N_7_5 = 7.5;
		public static final double N_8 = 8;

		// Angles in radians
		public static final double DEG_15 = Math.toRadians(15);
		public static final double DEG_20 = Math.toRadians(20);
		public static final double DEG_30 = Math.toRadians(30);
		public static final double DEG_35 = Math.toRadians(35);
		public static final double DEG_40 = Math.toRadians(40);
		public static final double DEG_45 = Math.toRadians(45);
		public static final double DEG_55 = Math.toRadians(55);
		public static final double DEG_90 = Math.toRadians(90);
		public static final double DEG_180 = Math.toRadians(180);
		public static final double DEG_360 = Math.toRadians(360);

		// Constraint for the motion profiled robot angle controller
		public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS
			= new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
			MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
	}

	public static final class NeoMotorConstants {
		public static final double FREE_SPEED_RPM = 5676;
	}
	public static final class VisionConstants {
		public static final double MAX_SPEED_METERS_PER_SECOND = 0.2;
		public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI / 20;

		public static final double SPEAKER_TRANSLATIONAL_ACCEL_CONSTANT = 1.5;
		public static final double SPEAKER_ROTATIONAL_ACCEL_CONSTANT = 1;
		public static final double X_MARGIN_TO_SPEAKER = 0.03;
		public static final double Y_MARGIN_TO_SPEAKER = 0.03;
		public static final double ROT_MARGIN_TO_SPEAKER = 0.04;
		public static final double SPEAKER_TARGET_DISTANCE = 1.2;

		public static final double SOURCE_TRANSLATIONAL_ACCEL_CONSTANT = 1.5;
		public static final double SOURCE_ROTATIONAL_ACCEL_CONSTANT = 1;
		public static final double Y_MARGIN_TO_SOURCE = 0.03;
		public static final double ROT_MARGIN_TO_SOURCE = 0.04;
		public static final double SOURCE_DRIVE_FORWARD_POWER = 0.35;


		public static final double NOTE_TRANSLATIONAL_ACCEL_CONSTANT = 5;
		public static final double NOTE_ROTATIONAL_ACCEL_CONSTANT = 70;
		public static final double X_MARGIN_TO_NOTE = 0.03;
		public static final double Y_MARGIN_TO_NOTE = 0.03;
		public static final double ROT_MARGIN_TO_NOTE = 3;
		public static final double NOTE_TARGET_DISTANCE = 1.2;

		public static final double UNABLE_TO_SEE_TAG_CONSTANT = 4000;
		public static final double UNABLE_TO_SEE_NOTE_CONSTANT = 5000;

		public static final int RED_SPEAKER_TAG_ID = 4;
		public static final int BLUE_SPEAKER_TAG_ID = 7;
		public static final int RED_SOURCE_TAG1_ID = 9;
		public static final int RED_SOURCE_TAG2_ID = 10;
		public static final int BLUE_SOURCE_TAG1_ID = 1;
		public static final int BLUE_SOURCE_TAG2_ID = 2;
		public static final int RED_AMP_TAG_ID = 5;
		public static final int BLUE_AMP_TAG_ID = 6;

		public static final double SOURCE_TAG_ANGLE_DEGREES = 60.0;
		public static final double SPEAKER_TAG_ANGLE_DEGREES = 180.0;
		public static final double NOTE_ANGLE_DEGREES = 0.0;

		public static final int DRIVER_CAM_WIDTH_PIXELS = 1280;
		public static final int DRIVER_CAM_HEIGHT_PIXELS = 720;
	}
}
