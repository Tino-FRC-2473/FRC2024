package frc.robot.systems;

// Third party Hardware Imports
import com.kauailabs.navx.frc.AHRS;
import java.util.ArrayList;
//import com.revrobotics.CANSparkMax;

// WPILib Imports
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;
import frc.robot.utils.SwerveUtils;
import frc.robot.HardwareMap;
import frc.robot.RaspberryPI;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.SwerveConstants.AutoConstants;
import frc.robot.AutoPathChooser;
import frc.robot.SwerveConstants.VisionConstants;

public class DriveFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
		ALIGN_TO_SPEAKER_STATE,
		ALIGN_TO_SOURCE_STATE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;
	private int currentPointInPath;
	private boolean blueAlliance;
	private Double[] tagOrientationAngles;
	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	// The gyro sensor
	private AHRS gyro = new AHRS(SPI.Port.kMXP);
	private Timer timer = new Timer();

	// The raspberry pi
	private RaspberryPI rpi = new RaspberryPI();

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

	private int lockedSourceId;
	private int lockedSpeakerId;
	private boolean isSourceAligned;
	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		gyro = new AHRS(SPI.Port.kMXP);
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
	 * Returns the current autonomus point the robot is at.
	 *
	 * @return current point in path.
	 */
	public int getCurrentPointInPath() {
		return currentPointInPath;
	}

	/**
	 * Sets the current autonomus point the robot is at.
	 *
	 * @param point current point in path.
	 */
	public void setCurrentPointInPath(int point) {
		currentPointInPath = point;
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
		gyro.reset();
		resetOdometry(new Pose2d());
		if (AutoPathChooser.getAutoPathChooser() != null) {
			blueAlliance = AutoPathChooser.getSelectedAlliance();
			if (blueAlliance) {
				tagOrientationAngles = new Double[]
					{null, 315.0, 315.0, null, null, null, 90.0, 180.0,
					180.0, null, null, null, null, null, null, null, null};
			} else {
				tagOrientationAngles = new Double[]
					{null, null, null, 180.0, 180.0, 270.0, null, null,
					null, 45.0, 45.0, null, null, null, null, null, null};
			}
		}
		lockedSourceId = -1;
		lockedSpeakerId = -1;
		isSourceAligned = false;
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
		gyro.reset();
		resetOdometry(new Pose2d());
		if (AutoPathChooser.getAutoPathChooser() != null) {
			blueAlliance = AutoPathChooser.getSelectedAlliance();
			if (blueAlliance) {
				tagOrientationAngles = new Double[]
					{null, 315.0, 315.0, null, null, null, 90.0, 180.0,
					180.0, null, null, null, null, null, null, null, null};
			} else {
				tagOrientationAngles = new Double[]
					{null, null, null, 180.0, 180.0, 270.0, null, null,
					null, 45.0, 45.0, null, null, null, null, null, null};
			}
		}
		lockedSourceId = -1;
		lockedSpeakerId = -1;
		isSourceAligned = false;
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
		SmartDashboard.putNumber("Heading", getPose().getRotation().getDegrees());
		SmartDashboard.putNumber("Auto point #", currentPointInPath);

		/*
		Auto Path Points
		Refer to the following document for point references
		Link: http://tinyurl.com/2023-auto-planning
		GD Dir: (FRC > 23-24 > 24 Season > Autonomous > Autonomous Path Planning)
		*/
		switch (autoState) {
			case DRIVE_PATH_1:
				ArrayList<Pose2d> path1Points = new ArrayList<Pose2d>();
				if (blueAlliance) {
					path1Points.add(new Pose2d(-1, AutoConstants.N_3,
						new Rotation2d(Math.toRadians(0))));
					path1Points.add(new Pose2d(-AutoConstants.N_3_5, AutoConstants.N_5,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_90))));
					path1Points.add(new Pose2d(-AutoConstants.N_6, AutoConstants.N_5,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_180))));
				} else {
					path1Points.add(new Pose2d(-1, -AutoConstants.N_3,
						new Rotation2d(Math.toRadians(0))));
					path1Points.add(new Pose2d(-AutoConstants.N_3_5, -AutoConstants.N_5,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_90))));
					path1Points.add(new Pose2d(-AutoConstants.N_6, -AutoConstants.N_5,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_180))));
				}
				return driveAlongPath(path1Points);
			case DRIVE_PATH_2:
				ArrayList<Pose2d> path2Points = new ArrayList<Pose2d>();
				if (blueAlliance) {
					path2Points.add(new Pose2d(0, 0,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_45))));
					path2Points.add(new Pose2d(-AutoConstants.N_3_5, AutoConstants.N_4,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_90))));
					path2Points.add(new Pose2d(-AutoConstants.N_6, AutoConstants.N_4,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_180))));
				} else {
					path2Points.add(new Pose2d(0, 0,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_45))));
					path2Points.add(new Pose2d(-AutoConstants.N_3_5, -AutoConstants.N_4,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_90))));
					path2Points.add(new Pose2d(-AutoConstants.N_6, -AutoConstants.N_4,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_180))));
				}
				return driveAlongPath(path2Points);
			case DRIVE_PATH_3:
				ArrayList<Pose2d> path3Points = new ArrayList<Pose2d>();
				if (blueAlliance) {
					path3Points.add(new Pose2d(0, 0,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_45))));
					path3Points.add(new Pose2d(-AutoConstants.N_5, 0,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_90))));
					path3Points.add(new Pose2d(-AutoConstants.N_6, AutoConstants.N_5,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_180))));
				} else {
					path3Points.add(new Pose2d(0, 0,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_45))));
					path3Points.add(new Pose2d(-AutoConstants.N_5, 0,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_90))));
					path3Points.add(new Pose2d(-AutoConstants.N_6, -AutoConstants.N_5,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_180))));
				}
				return driveAlongPath(path3Points);
			case DRIVE_PATH_4_STATE_1:
				ArrayList<Pose2d> path4Points1 = new ArrayList<Pose2d>();
				if (blueAlliance) {
					path4Points1.add(new Pose2d(0, -AutoConstants.N_2,
						new Rotation2d(Math.toRadians(0))));
				} else {
					path4Points1.add(new Pose2d(0, 2, new Rotation2d(Math.toRadians(0))));
				}
				return driveAlongPath(path4Points1);
			case DRIVE_PATH_4_STATE_2:
				ArrayList<Pose2d> path4Points2 = new ArrayList<Pose2d>();
				if (blueAlliance) {
					path4Points2.add(new Pose2d(-1, -1, new Rotation2d(Math.toRadians(0))));
					path4Points2.add(new Pose2d(-AutoConstants.N_3_5, AutoConstants.N_3,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_90))));
					path4Points2.add(new Pose2d(-AutoConstants.N_6, AutoConstants.N_3,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_180))));
				} else {
					path4Points2.add(new Pose2d(-1, 1, new Rotation2d(Math.toRadians(0))));
					path4Points2.add(new Pose2d(-AutoConstants.N_3_5, -AutoConstants.N_5,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_90))));
					path4Points2.add(new Pose2d(-AutoConstants.N_6, -AutoConstants.N_3,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_180))));
				}
				return driveAlongPath(path4Points2);
			case DRIVE_PATH_5:
				ArrayList<Pose2d> path5Points = new ArrayList<Pose2d>();
				if (blueAlliance) {
					path5Points.add(new Pose2d(-AutoConstants.N_6, 0,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_180))));
				} else {
					path5Points.add(new Pose2d(-AutoConstants.N_6, 0,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_180))));
				}
				return driveAlongPath(path5Points);
			case PENDING:
				timer.start();
				return pause(AutoConstants.WAIT_TIME);
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
		SmartDashboard.putNumber("Heading", getPose().getRotation().getDegrees());

		rpi.update();
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
					resetOdometry(new Pose2d(new Translation2d(getPose().getX(), getPose().getY()),
						new Rotation2d(0)));
				}
				break;

			case ALIGN_TO_SOURCE_STATE:
				if (lockedSourceId == -1) {
					if (blueAlliance) {
						//id 1 and 2
						double z1 = rpi.getAprilTagZInv(1);
						double z2 = rpi.getAprilTagZInv(2);
						if (!(z1 == 4000 && z2 == 4000)) {
							if (z1 == 4000) {
								lockedSourceId = 2;
							} else if (z2 == 4000){
								lockedSourceId = 1;
							} else {
								if (z1 <= z2) {
									lockedSourceId = 1;
								} else {
									lockedSourceId = 2;
								}
							}
						}
					} else {
						//id 9 and 10
						double z9 = rpi.getAprilTagZInv(9);
						double z10 = rpi.getAprilTagZInv(10);
						if (!(z9 == 4000 && z10 == 4000)) {
							if (z9 == 4000) {
								lockedSourceId = 10;
							} else if (z10 == 4000){
								lockedSourceId = 9;
							} else {
								if (z9 <= z10) {
									lockedSourceId = 9;
								} else {
									lockedSourceId = 10;
								}
							}
						}
					}
				} else {
					alignToSource(lockedSourceId);
				}

				break;

			case ALIGN_TO_SPEAKER_STATE:
				if (lockedSpeakerId == -1) {
					if (blueAlliance) {
						//id 7
						if (rpi.getAprilTagZInv(7)!=4000) {
							lockedSpeakerId = 7;
						}
					} else {
						//id 4
						if (rpi.getAprilTagZInv(4)!=4000) {
							lockedSpeakerId = 4;
						}
					}
				} else {
					alignToSpeaker(lockedSpeakerId);
				}

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
				if (input.isCircleButtonPressed()) {
					return FSMState.ALIGN_TO_SPEAKER_STATE;
				} else if (input.isTriangleButtonPressed()) {
					return FSMState.ALIGN_TO_SOURCE_STATE;
				}
				return FSMState.TELEOP_STATE;

			case ALIGN_TO_SOURCE_STATE:
				if (input.isTriangleButtonReleased()) {
					lockedSourceId = -1;
					isSourceAligned = false;
					return FSMState.TELEOP_STATE;
				}
				return FSMState.ALIGN_TO_SOURCE_STATE;

			case ALIGN_TO_SPEAKER_STATE:
				if (input.isCircleButtonReleased()) {
					lockedSpeakerId = -1;
					return FSMState.TELEOP_STATE;
				}
				return FSMState.ALIGN_TO_SPEAKER_STATE;

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

		SmartDashboard.putNumber("x diff", xDiff);
		SmartDashboard.putNumber("y diff", yDiff);
		SmartDashboard.putNumber("a diff", aDiff);

		double xSpeed;
		double ySpeed;
		if (Math.abs(xDiff) > Math.abs(yDiff)) {
			xSpeed = clamp(xDiff / AutoConstants.AUTO_DRIVE_TRANSLATIONAL_SPEED_ACCEL_CONSTANT,
			-AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_SPEED_METERS_PER_SECOND);
			ySpeed = xSpeed * (yDiff / xDiff);
			if (Math.abs(xDiff) < AutoConstants.CONSTANT_SPEED_THRESHOLD && Math.abs(yDiff)
				< AutoConstants.CONSTANT_SPEED_THRESHOLD) {
				xSpeed = (AutoConstants.CONSTANT_SPEED_THRESHOLD * xDiff / Math.abs(xDiff))
					/ AutoConstants.AUTO_DRIVE_TRANSLATIONAL_SPEED_ACCEL_CONSTANT;
				ySpeed = xSpeed * (yDiff / xDiff);
			}
		} else {
			ySpeed = clamp(yDiff / AutoConstants.AUTO_DRIVE_TRANSLATIONAL_SPEED_ACCEL_CONSTANT,
			-AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_SPEED_METERS_PER_SECOND);
			xSpeed = ySpeed * (xDiff / yDiff);
			if (Math.abs(xDiff) < AutoConstants.CONSTANT_SPEED_THRESHOLD && Math.abs(yDiff)
				< AutoConstants.CONSTANT_SPEED_THRESHOLD) {
				ySpeed = (AutoConstants.CONSTANT_SPEED_THRESHOLD * yDiff / Math.abs(yDiff))
					/ AutoConstants.AUTO_DRIVE_TRANSLATIONAL_SPEED_ACCEL_CONSTANT;
				xSpeed = ySpeed * (xDiff / yDiff);
			}
		}

		xSpeed = Math.abs(xDiff) > AutoConstants.AUTO_DRIVE_METERS_MARGIN_OF_ERROR
			? xSpeed : 0;
		ySpeed = Math.abs(yDiff) > AutoConstants.AUTO_DRIVE_METERS_MARGIN_OF_ERROR
			? ySpeed : 0;
		double aSpeed = Math.abs(aDiff) > AutoConstants.AUTO_DRIVE_DEGREES_MARGIN_OF_ERROR
			? (aDiff > 0 ? Math.min(AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, aDiff
			/ AutoConstants.AUTO_DRIVE_ANGULAR_SPEED_ACCEL_CONSTANT) : Math.max(
			-AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, aDiff
			/ AutoConstants.AUTO_DRIVE_ANGULAR_SPEED_ACCEL_CONSTANT)) : 0;

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
			drive(0, 0, 0, true, false);
			return true;
		}
		if (driveToPose(points.get(currentPointInPath))) {
			currentPointInPath++;
		}
		return false;
	}

	/**
	 * @param rotFinal a number proportional to the angle from the camera to the tag
	 * Turns the robot towards the source's april tag and drives
	 * towards it and straightens the robot when driving against the wall. This positions
	 * robot to be at the source to pickup a note.
	 */
	public void alignToSource(int id) {
		double yDiff = rpi.getAprilTagX(id);
		double xDiff = rpi.getAprilTagZ(id) - 0.7;
		double aDiff = rpi.getAprilTagXInv(id);

		SmartDashboard.putNumber("x diff", xDiff);
		SmartDashboard.putNumber("y diff", yDiff);
		SmartDashboard.putNumber("a diff", aDiff);

		double xSpeed = clamp(xDiff
			/ 3,
			-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
			VisionConstants.MAX_SPEED_METERS_PER_SECOND);
		double ySpeed = clamp(yDiff
			/ 3,
			-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
			VisionConstants.MAX_SPEED_METERS_PER_SECOND);
		double aSpeed = -clamp(aDiff / 2,
			-VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
			VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);

		double xSpeedField = Math.abs(xDiff) > 0.02
			? (xSpeed * Math.cos(Math.toRadians(tagOrientationAngles[id])))
			+ (ySpeed * Math.sin(Math.toRadians(tagOrientationAngles[id]))) : 0;
		double ySpeedField = Math.abs(yDiff) > 0.02
			? (ySpeed * Math.cos(Math.toRadians(tagOrientationAngles[id])))
			- (xSpeed * Math.sin(Math.toRadians(tagOrientationAngles[id]))) : 0;
		aSpeed = Math.abs(aDiff) > 0.01
			? aSpeed : 0;

		if (!isSourceAligned) {
			drive(xSpeedField, ySpeedField, aSpeed, true, false);
			if (xSpeedField == 0 && ySpeedField == 0 && aSpeed == 0) {
				isSourceAligned = true;
			}
		} else {
			drive(0.1 * Math.cos(Math.toRadians(tagOrientationAngles[id])),
				-0.1 * Math.sin(Math.toRadians(tagOrientationAngles[id])),
				aSpeed, true, false);
		}
	}

	/**
	 * @param rotFinal a number proportional to the angle from the camera to the tag
	 * @param dist the distance in meters from the camera to the tag
	 * Turns the robot towards the speaker's april tag and drives it
	 * forward to be a set distance away from the speaker. This positions
	 * robot to be at the ideal shooting position relative to the speaker.
	 */
	public void alignToSpeaker(int id) {
		double yDiff = rpi.getAprilTagX(id);
		double xDiff = rpi.getAprilTagZ(id) - 0.7;
		double aDiff = rpi.getAprilTagXInv(id);

		SmartDashboard.putNumber("x diff", xDiff);
		SmartDashboard.putNumber("y diff", yDiff);
		SmartDashboard.putNumber("a diff", aDiff);

		double xSpeed = clamp(xDiff
			/ 3,
			-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
			VisionConstants.MAX_SPEED_METERS_PER_SECOND);
		double ySpeed = clamp(yDiff
			/ 3,
			-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
			VisionConstants.MAX_SPEED_METERS_PER_SECOND);
		double aSpeed = -clamp(aDiff / 2,
			-VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
			VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);

		double xSpeedField = Math.abs(xDiff) > 0.02
			? (xSpeed * Math.cos(Math.toRadians(tagOrientationAngles[id])))
			+ (ySpeed * Math.sin(Math.toRadians(tagOrientationAngles[id]))) : 0;
		double ySpeedField = Math.abs(yDiff) > 0.02
			? (ySpeed * Math.cos(Math.toRadians(tagOrientationAngles[id])))
			- (xSpeed * Math.sin(Math.toRadians(tagOrientationAngles[id]))) : 0;
		aSpeed = Math.abs(aDiff) > 0.01
			? aSpeed : 0;

		drive(xSpeedField, ySpeedField, aSpeed, true, false);
	}

	// public boolean driveToTag(int id) {
	// 	double xDiff = rpi.getAprilTagX(id);
	// 	double yDiff = rpi.getAprilTagZ(id) - 0.7;
	// 	double aDiff = rpi.getAprilTagXInv(id);

	// 	if (xDiff == 4000) {
	// 		drive(0,0,0, false, false);
	// 	}
	// 	SmartDashboard.putNumber("x diff", xDiff);
	// 	SmartDashboard.putNumber("y diff", yDiff);
	// 	SmartDashboard.putNumber("a diff", aDiff);

	// 	double xSpeed = clamp(xDiff
	// 		/ 3,
	// 		-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
	// 		VisionConstants.MAX_SPEED_METERS_PER_SECOND);
	// 	double ySpeed = clamp(yDiff
	// 		/ 3,
	// 		-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
	// 		VisionConstants.MAX_SPEED_METERS_PER_SECOND);
	// 	double aSpeed = -clamp(aDiff / 2,
	// 		-VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
	// 		VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);

	// 	xSpeed = Math.abs(xDiff) > 0.02
	// 		? xSpeed : 0;
	// 	ySpeed = Math.abs(yDiff) > 0.02
	// 		? ySpeed : 0;
	// 	aSpeed = Math.abs(aDiff) > 0.01
	// 		? aSpeed : 0;

	// 	drive(ySpeed, xSpeed, aSpeed, true, false);
	// 	if (xSpeed == 0 && aSpeed == 0) {
	// 		return true;
	// 	}
	// 	return false;
	// }

	/**
	 * Drives the robot until it reaches a given object.
	 * @param seconds seconds to wait
	 * @return whether or not the wait time has been completed
	 */
	public boolean pause(double seconds) {
		if (timer.get() >= seconds) {
			timer.stop();
			return true;
		}
		return false;
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
