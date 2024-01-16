package frc.robot.systems;

// Third party Hardware Imports
import com.kauailabs.navx.frc.AHRS;
import java.util.ArrayList;
//import com.revrobotics.CANSparkMax;

// WPILib Imports
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;
import frc.robot.utils.SwerveUtils;
import frc.robot.HardwareMap;
import frc.robot.RaspberryPI;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.SwerveConstants.AutoConstants;

public class DriveFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
		ALIGN_TO_TAG_STATE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;
	private int currentPointInPath;
	private double lastSeenTagYaw;
	private double lastSeenTagX;
	private double lastSeenTagY;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	// The gyro sensor
	private AHRS gyro = new AHRS(SPI.Port.kMXP);

	private RaspberryPI rpi;

	// Slew rate filter variables for controlling lateral acceleration
	private double currentRotation = 0.0;
	private double currentTranslationDir = 0.0;
	private double currentTranslationMag = 0.0;

	private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE);
	private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.ROTATIONAL_SLEW_RATE);
	private double prevTime = WPIUtilJNI.now() * DriveConstants.TIME_CONSTANT;

	// Create MAXSwerveModules
	private final MAXSwerveModule frontLeft = new MAXSwerveModule(
		HardwareMap.FRONT_LEFT_DRIVING_CAN_ID,
		HardwareMap.FRONT_LEFT_TURNING_CAN_ID,
		DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

	private final MAXSwerveModule frontRight = new MAXSwerveModule(
		HardwareMap.FRONT_RIGHT_DRIVING_CAN_ID,
		HardwareMap.FRONT_RIGHT_TURNING_CAN_ID,
		DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

	private final MAXSwerveModule rearLeft = new MAXSwerveModule(
		HardwareMap.REAR_LEFT_DRIVING_CAN_ID,
		HardwareMap.REAR_LEFT_TURNING_CAN_ID,
		DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET);

	private final MAXSwerveModule rearRight = new MAXSwerveModule(
		HardwareMap.REAR_RIGHT_DRIVING_CAN_ID,
		HardwareMap.REAR_RIGHT_TURNING_CAN_ID,
		DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET);

	// Odometry class for tracking robot pose
	private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
		DriveConstants.DRIVE_KINEMATICS,
		Rotation2d.fromDegrees(-gyro.getAngle()),
		new SwerveModulePosition[] {
			frontLeft.getPosition(),
			frontRight.getPosition(),
			rearLeft.getPosition(),
			rearRight.getPosition()
		});

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init
		gyro = new AHRS(SPI.Port.kMXP);
		rpi = new RaspberryPI();

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */

	public void reset() {
		currentState = FSMState.TELEOP_STATE;

		lastSeenTagX = 4000;
		lastSeenTagY = 4000;
		lastSeenTagYaw = 4000;

		resetOdometry(new Pose2d());
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */

	public void resetAutonomus() {
		currentPointInPath = 0;

		lastSeenTagX = 4000;
		lastSeenTagY = 4000;
		lastSeenTagYaw = 4000;

		resetOdometry(new Pose2d());
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Performs specific action based on the autoState passed in.
	 * @param autoState autoState that the subsystem executes.
	 * @return if the action carried out in this state has finished executing
	 */
	public boolean updateAutonomous(AutoFSMState autoState) {
		odometry.update(Rotation2d.fromDegrees(-gyro.getAngle()),
			new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()});

		SmartDashboard.putNumber("X Pos", getPose().getX());
		SmartDashboard.putNumber("Y Pos", getPose().getY());
		SmartDashboard.putNumber("Heading", getHeading());

		if (rpi.getAprilTagX(1) != 4000 && rpi.getAprilTagY(1) != 4000 && rpi.getAprilTagYaw(1) != 4000) {
			lastSeenTagX = rpi.getAprilTagX(1);
			lastSeenTagY = rpi.getAprilTagY(1);
			lastSeenTagYaw = rpi.getAprilTagYaw(1);
			SmartDashboard.putBoolean("Can See Tag", true);
		} else {
			SmartDashboard.putBoolean("Can See Tag", false);
		}

		switch (autoState) {
			case AUTO_STATE:
				ArrayList<Pose2d> points = new ArrayList<Pose2d>();
				// points.add(new Pose2d(0, -2, new Rotation2d(Math.toRadians(90))));
				// points.add(new Pose2d(-2, -2, new Rotation2d(Math.toRadians(180))));
				// points.add(new Pose2d(-2, 0, new Rotation2d(Math.toRadians(-90))));
				// points.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
				return driveAlongPath(points);
			default:
				return false;
		}
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(
			Rotation2d.fromDegrees(-gyro.getAngle()),
				new SwerveModulePosition[] {
					frontLeft.getPosition(),
					frontRight.getPosition(),
					rearLeft.getPosition(),
					rearRight.getPosition()
				},
				pose);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		odometry.update(Rotation2d.fromDegrees(-gyro.getAngle()),
			new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()});

		SmartDashboard.putNumber("X Pos", getPose().getX());
		SmartDashboard.putNumber("Y Pos", getPose().getY());
		SmartDashboard.putNumber("Heading", getHeading());


		if (rpi.getAprilTagX(1) != 4000 && rpi.getAprilTagY(1) != 4000 && rpi.getAprilTagYaw(1) != 4000) {
			lastSeenTagX = rpi.getAprilTagX(1);
			lastSeenTagY = rpi.getAprilTagY(1);
			lastSeenTagYaw = rpi.getAprilTagYaw(1);
			SmartDashboard.putBoolean("Can See Tag", true);
		} else {
			SmartDashboard.putBoolean("Can See Tag", false);
		}

		SmartDashboard.putNumber("last seen x", lastSeenTagX);
		SmartDashboard.putNumber("last seen y", lastSeenTagY);
		SmartDashboard.putNumber("last seen yaw", lastSeenTagYaw);

		if (input == null) {
			return;
		}
		switch (currentState) {
			case TELEOP_STATE:
				drive(-MathUtil.applyDeadband((input.getControllerLeftJoystickY()
					* Math.abs(input.getControllerLeftJoystickY())),
					OIConstants.DRIVE_DEADBAND),
					-MathUtil.applyDeadband((input.getControllerLeftJoystickX()
					* Math.abs(input.getControllerLeftJoystickX())),
					OIConstants.DRIVE_DEADBAND),
					-MathUtil.applyDeadband(input.getControllerRightJoystickX(),
					OIConstants.DRIVE_DEADBAND), true, true);
				if (input.isBackButtonPressed()) {
					gyro.reset();
				}
				break;

			case ALIGN_TO_TAG_STATE:
				//System.out.println("x tag: " + rpi.getAprilTagX(1));
				driveToTag();
				// LEAVE ROT FINAL AS ZERO (ANGLE OF APRIL TAG)
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case TELEOP_STATE:
				lastSeenTagX = 4000;
				lastSeenTagY = 4000;
				lastSeenTagYaw = 4000;
				if (input.isCircleButtonPressed()) {
					return FSMState.ALIGN_TO_TAG_STATE;
				}
				return FSMState.TELEOP_STATE;
			case ALIGN_TO_TAG_STATE:
				if (input.isCircleButtonReleased()) {
					return FSMState.TELEOP_STATE;
				}
				return FSMState.ALIGN_TO_TAG_STATE;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 * @param rateLimit     Whether to enable rate limiting for smoother control.
	 */
	public void drive(double xSpeed, double ySpeed, double rot,
		boolean fieldRelative, boolean rateLimit) {

		double xSpeedCommanded;
		double ySpeedCommanded;

		if (rateLimit) {
			// Convert XY to polar for rate limiting
			double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
			double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

			// Calculate the direction slew rate based on an estimate of the lateral acceleration
			double directionSlewRate;
			if (currentTranslationMag != 0.0) {
				directionSlewRate = Math.abs(DriveConstants.DIRECTION_SLEW_RATE
					/ currentTranslationMag);
			} else {
				directionSlewRate = DriveConstants.INSTANTANEOUS_SLEW_RATE;
				//some high number that means the slewrate is effectively instantaneous
			}

			double currentTime = WPIUtilJNI.now() * DriveConstants.TIME_CONSTANT;
			double elapsedTime = currentTime - prevTime;
			double angleDif = SwerveUtils.angleDifference(inputTranslationDir,
				currentTranslationDir);

			if (angleDif < DriveConstants.ANGLE_MULTIPLIER_1 * Math.PI) {
				currentTranslationDir = SwerveUtils.stepTowardsCircular(currentTranslationDir,
					inputTranslationDir, directionSlewRate * elapsedTime);
				currentTranslationMag = magLimiter.calculate(inputTranslationMag);
			} else if (angleDif > DriveConstants.ANGLE_MULTIPLIER_2 * Math.PI) {
				if (currentTranslationMag > DriveConstants.CURRENT_THRESHOLD) {
					// some small number to avoid floating-point errors with equality checking
					// keep currentTranslationDir unchanged
					currentTranslationMag = magLimiter.calculate(0.0);
				} else {
					currentTranslationDir = SwerveUtils.wrapAngle(currentTranslationDir + Math.PI);
					currentTranslationMag = magLimiter.calculate(inputTranslationMag);
				}
			} else {
				currentTranslationDir = SwerveUtils.stepTowardsCircular(currentTranslationDir,
					inputTranslationDir, directionSlewRate * elapsedTime);
				currentTranslationMag = magLimiter.calculate(0.0);
			}
			prevTime = currentTime;

			xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
			ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
			currentRotation = rotLimiter.calculate(rot);

		} else {
			xSpeedCommanded = xSpeed;
			ySpeedCommanded = ySpeed;
			currentRotation = rot;
		}

		// Convert the commanded speeds into the correct units for the drivetrain
		double xSpeedDelivered = xSpeedCommanded * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
		double ySpeedDelivered = ySpeedCommanded * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
		double rotDelivered = currentRotation * DriveConstants.MAX_ANGULAR_SPEED;

		var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered,
					rotDelivered, Rotation2d.fromDegrees(-gyro.getAngle()))
				: new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
		SwerveDriveKinematics.desaturateWheelSpeeds(
			swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

		frontLeft.setDesiredState(swerveModuleStates[0]);
		frontRight.setDesiredState(swerveModuleStates[1]);
		rearLeft.setDesiredState(swerveModuleStates[2]);
		rearRight.setDesiredState(swerveModuleStates[(2 + 1)]);
	}

	/**
	 * Drives the robot to a final odometry state.
	 * @param pose final odometry position for the robot
	 * @return if the robot has driven to the current position
	 */
	public boolean driveToPose(Pose2d pose) {
		double x = pose.getX();
		double y = pose.getY();
		double angle = pose.getRotation().getDegrees();

		double xDiff = x - getPose().getX();
		double yDiff = y - getPose().getY();
		double aDiff = angle - getPose().getRotation().getDegrees();

		System.out.println("Xdiff: " + xDiff);
		System.out.println("Ydiff: " + yDiff);
		System.out.println("adiff: " + aDiff);
		double xSpeed = Math.abs(xDiff) > AutoConstants.METERS_MARGIN_OF_ERROR
			? clamp(xDiff / AutoConstants.TRANSLATIONAL_SPEED_ACCEL_CONSTANT,
			-AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_SPEED_METERS_PER_SECOND) : 0;
		double ySpeed = Math.abs(yDiff) > AutoConstants.METERS_MARGIN_OF_ERROR
			? clamp(yDiff / AutoConstants.TRANSLATIONAL_SPEED_ACCEL_CONSTANT,
			-AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_SPEED_METERS_PER_SECOND) : 0;
		double aSpeed = Math.abs(aDiff) > AutoConstants.DEGREES_MARGIN_OF_ERROR ? (aDiff > 0
			? Math.min(AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, aDiff
			/ AutoConstants.ANGULAR_SPEED_ACCEL_CONSTANT) : Math.max(
			-AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, aDiff
			/ AutoConstants.ANGULAR_SPEED_ACCEL_CONSTANT)) : 0;

		drive(xSpeed, ySpeed, aSpeed, true, false);
		if (xSpeed == 0 && ySpeed == 0 && aSpeed == 0) {
			return true;
		}
		return false;
	}

	/**
	 * Drives the robot through a series of points.
	 * @param points arraylist of points to drive to
	 * @return if the robot has driven to the next position
	 */
	public boolean driveAlongPath(ArrayList<Pose2d> points) {
		if (currentPointInPath >= points.size()) {
			return true;
		}
		if (driveToPose(points.get(currentPointInPath))) {
			currentPointInPath++;
		}
		return false;
	}

	/**
	 * Drives the robot until it reaches a given position relative to the apriltag.
	 */
	public void driveToTag() {
		if (lastSeenTagX == 4000 || lastSeenTagY == 4000 || lastSeenTagYaw == 4000) {
			drive(0, 0, 0, false, false);
			return;
		}
		double transformedXDist = Math.hypot(lastSeenTagX, lastSeenTagX)
			* Math.sin(Math.toRadians(-(getHeading() + lastSeenTagYaw)));
		double transformedYDist = Math.hypot(lastSeenTagX, lastSeenTagY)
			* Math.cos(Math.toRadians(-(getHeading() + lastSeenTagYaw)));

		double xSpeed = clamp(-transformedXDist / AutoConstants.DRIVE_TO_TAG_TRANSLATIONAL_CONSTANT,
			-AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_SPEED_METERS_PER_SECOND);
		double ySpeed = clamp(transformedYDist / AutoConstants.DRIVE_TO_TAG_TRANSLATIONAL_CONSTANT,
			-AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_SPEED_METERS_PER_SECOND);
		double rotSpeed = clamp(-lastSeenTagYaw / AutoConstants.DRIVE_TO_TAG_ROTATIONAL_CONSTANT,
			-AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
			AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);

		if (Math.abs(transformedXDist) < 3 && Math.abs(transformedYDist) < 20 && Math.abs(lastSeenTagYaw) < 5) {
			System.out.println("aligned");
			drive(0, 0, 0, true, false);
		} else {
			drive(ySpeed, xSpeed, 0, true, false);
		}
	}

	/**
	 * Drives the robot until it reaches a given object.
	 * @param dist constantly updating distance to the object
	 * @param rotFinal constantly updating angular difference to the point
	 */
	public void driveUntilObject(double dist, double rotFinal) {
		double power = clamp((dist - AutoConstants.DISTANCE_MARGIN_TO_DRIVE_TO_OBJECT)
			/ AutoConstants.DRIVE_TO_OBJECT_TRANSLATIONAL_CONSTANT,
			-AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_SPEED_METERS_PER_SECOND);
		double rotSpeed = clamp(-rotFinal / AutoConstants.DRIVE_TO_OBJECT_ROTATIONAL_CONSTANT,
			-AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
			AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);

		// System.out.println("power: " + power);
		// System.out.println("rotSpeed: " + rotSpeed);

		if ((dist < AutoConstants.DISTANCE_MARGIN_TO_DRIVE_TO_OBJECT && Math.abs(rotFinal)
			<= AutoConstants.ANGLE_MARGIN_TO_DRIVE_TO_OBJECT) || (dist == -1 || rotFinal == -1)) {
			drive(0, 0, 0, false, false);
		// } else if (Math.abs(rotFinal) > 5) {
		// 	drive(0, 0, rotSpeed, false, false);
		} else {
			drive(power, 0, rotSpeed, false, false);
		}
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
			desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		rearLeft.setDesiredState(desiredStates[2]);
		rearRight.setDesiredState(desiredStates[2 + 1]);
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		frontLeft.resetEncoders();
		rearLeft.resetEncoders();
		frontRight.resetEncoders();
		rearRight.resetEncoders();
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		gyro.reset();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return Rotation2d.fromDegrees(-gyro.getAngle()).getDegrees();
	}

	/**
	 * Clamps a double between two values.
	 * @param value the value wished to be bounded
	 * @param lowerBound the lower limit for the value
	 * @param upperBound the upper limit for the value
	 * @return the clamped value of the double
	 */
	public static double clamp(double value, double lowerBound, double upperBound) {
		return Math.min(Math.max(value, lowerBound), upperBound);
	}
}
