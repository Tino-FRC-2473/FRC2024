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
	private int startingPos;
	private boolean svrMech;

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
		if (AutoPathChooser.getAllianceChooser() != null) {
			blueAlliance = AutoPathChooser.getSelectedAlliance();
		} else {
			blueAlliance = true;
		}
		if (blueAlliance) {
			tagOrientationAngles = new Double[]
				{null, VisionConstants.SOURCE_TAG_ANGLE_DEGREES,
					VisionConstants.SOURCE_TAG_ANGLE_DEGREES, null,
					null, null, null, VisionConstants.SPEAKER_TAG_ANGLE_DEGREES,
					VisionConstants.SPEAKER_TAG_ANGLE_DEGREES, null, null, null,
					null, null, null, null, null};
		} else {
			tagOrientationAngles = new Double[]
				{null, null, null, VisionConstants.SPEAKER_TAG_ANGLE_DEGREES,
					VisionConstants.SPEAKER_TAG_ANGLE_DEGREES, null, null, null, null,
					-VisionConstants.SOURCE_TAG_ANGLE_DEGREES,
					-VisionConstants.SOURCE_TAG_ANGLE_DEGREES, null, null, null, null, null,
					null};
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
		if (AutoPathChooser.getMechChooser() != null) {
			svrMech = AutoPathChooser.getSelectedMech();
		} else {
			svrMech = true;
		}
		if (AutoPathChooser.getAllianceChooser() != null) {
			blueAlliance = AutoPathChooser.getSelectedAlliance();
		} else {
			blueAlliance = true;
		}
		if (AutoPathChooser.getStartPosChooser() != null) {
			startingPos = AutoPathChooser.getStartingPos();
		} else {
			startingPos = 1;
		}
		if (blueAlliance) {
			tagOrientationAngles = new Double[]
				{null, VisionConstants.SOURCE_TAG_ANGLE_DEGREES,
					VisionConstants.SOURCE_TAG_ANGLE_DEGREES, null,
					null, null, null, VisionConstants.SPEAKER_TAG_ANGLE_DEGREES,
					VisionConstants.SPEAKER_TAG_ANGLE_DEGREES, null, null, null,
					null, null, null, null, null};
		} else {
			tagOrientationAngles = new Double[]
				{null, null, null, VisionConstants.SPEAKER_TAG_ANGLE_DEGREES,
					VisionConstants.SPEAKER_TAG_ANGLE_DEGREES, null, null, null, null,
					-VisionConstants.SOURCE_TAG_ANGLE_DEGREES,
					-VisionConstants.SOURCE_TAG_ANGLE_DEGREES, null, null, null, null, null,
					null};
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

		switch (autoState) {
			case LEAVE:
				ArrayList<Pose2d> leave = new ArrayList<Pose2d>();
				if (startingPos == 0) { // center
					leave.add(new Pose2d(-1, AutoConstants.N_3,
						new Rotation2d(Math.toRadians(0))));
					leave.add(new Pose2d(-AutoConstants.N_3_5, AutoConstants.N_5,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_90))));
					leave.add(new Pose2d(-AutoConstants.N_6, AutoConstants.N_5,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_180))));
				} else if (startingPos == 1) { // source side speaker
					leave.add(new Pose2d(-AutoConstants.N_3_5, AutoConstants.N_4,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_90))));
					leave.add(new Pose2d(-AutoConstants.N_6, AutoConstants.N_4,
						new Rotation2d(Math.toRadians(-AutoConstants.DEG_180))));
				} else if (startingPos == 2 || startingPos == 2 + 1) { // amp side speaker
					leave.add(new Pose2d(-AutoConstants.N_5, 0,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_90))));
					leave.add(new Pose2d(-AutoConstants.N_6, AutoConstants.N_5,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_180))));
				} else if (startingPos == 2 + 2) { // other (leave in a straight line)
					leave.add(new Pose2d(-AutoConstants.N_6, 0,
						new Rotation2d(Math.toRadians(AutoConstants.DEG_180))));
				} else {
					leave.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
				}
				return driveAlongPath(leave);
			case DRIVE_TO_SCORE:
				ArrayList<Pose2d> toScore = new ArrayList<Pose2d>();
				if (svrMech) {
					toScore.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
				} else {
					if (startingPos == 1) {
						toScore.add(new Pose2d(0, 0,
							new Rotation2d(Math.toRadians(-AutoConstants.DEG_45))));
					} else if (startingPos == 2) {
						toScore.add(new Pose2d(0, 0,
							new Rotation2d(Math.toRadians(AutoConstants.DEG_45))));
					} else if (startingPos == 2 + 1) {
						toScore.add(new Pose2d(-AutoConstants.N_0_5, -1,
							new Rotation2d(Math.toRadians(AutoConstants.DEG_90))));
					} else {
						toScore.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
					}
				}
				return driveAlongPath(toScore);
			case PICK_UP_1:
				ArrayList<Pose2d> pickUp1 = new ArrayList<Pose2d>();
				if (svrMech) {
					pickUp1.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
				} else {
					if (startingPos == 0) {
						pickUp1.add(new Pose2d(-1, 0, new Rotation2d(Math.toRadians(0))));
					} else if (startingPos == 1) {
						pickUp1.add(new Pose2d(-1, AutoConstants.N_0_5, new Rotation2d(
							Math.toRadians(0))));
					} else if (startingPos == 2) {
						pickUp1.add(new Pose2d(-1, -AutoConstants.N_0_5, new Rotation2d(
							Math.toRadians(0))));
					} else if (startingPos == 2 + 1) {
						pickUp1.add(new Pose2d(-AutoConstants.N_0_5, 0,
							new Rotation2d(Math.toRadians(-AutoConstants.DEG_90))));
					} else {
						pickUp1.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
					}
				}
				return driveAlongPath(pickUp1);
			case PICK_UP_2:
				ArrayList<Pose2d> pickUp2 = new ArrayList<Pose2d>();
				if (svrMech) {
					pickUp2.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
				} else {
					if (startingPos == 0) {
						pickUp2.add(new Pose2d(-1, AutoConstants.N_1_5,
							new Rotation2d(Math.toRadians(0))));
					} else if (startingPos == 1) {
						pickUp2.add(new Pose2d(-1, -1, new Rotation2d(Math.toRadians(0))));
					} else if (startingPos == 2) {
						pickUp2.add(new Pose2d(-1, 1, new Rotation2d(Math.toRadians(0))));
					} else if (startingPos == 2 + 1) {
						pickUp2.add(new Pose2d(-AutoConstants.N_0_5, AutoConstants.N_1_5,
							new Rotation2d(Math.toRadians(-AutoConstants.DEG_90))));
					} else {
						pickUp2.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
					}
				}
				return driveAlongPath(pickUp2);
			case PICK_UP_3:
				ArrayList<Pose2d> pickUp3 = new ArrayList<Pose2d>();
				if (svrMech) {
					pickUp3.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
				} else {
					if (startingPos == 0) {
						pickUp3.add(new Pose2d(-1, -AutoConstants.N_1_5,
							new Rotation2d(Math.toRadians(0))));
					} else if (startingPos == 1) {
						pickUp3.add(new Pose2d(-1, -AutoConstants.N_2_5,
							new Rotation2d(Math.toRadians(0))));
					} else if (startingPos == 2) {
						pickUp3.add(new Pose2d(-1, AutoConstants.N_2_5,
							new Rotation2d(Math.toRadians(0))));
					} else if (startingPos == 2 + 1) {
						pickUp3.add(new Pose2d(-AutoConstants.N_0_5, AutoConstants.N_2_5,
							new Rotation2d(Math.toRadians(-AutoConstants.DEG_90))));
					} else {
						pickUp3.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
					}
				}
				return driveAlongPath(pickUp3);
			case PICK_UP_4:
				ArrayList<Pose2d> pickUp4 = new ArrayList<Pose2d>();
				if (svrMech) {
					pickUp4.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
				} else if (blueAlliance) {
					if (startingPos == 0) {
						pickUp4.add(new Pose2d(-AutoConstants.N_6, -AutoConstants.N_2,
							new Rotation2d(Math.toRadians(0))));
					} else if (startingPos == 1) {
						pickUp4.add(new Pose2d(-AutoConstants.N_6, AutoConstants.N_3,
							new Rotation2d(Math.toRadians(0))));
					} else if (startingPos == 2) {
						pickUp4.add(new Pose2d(-AutoConstants.N_6, -1,
							new Rotation2d(Math.toRadians(0))));
					} else if (startingPos == 2 + 1) {
						pickUp4.add(new Pose2d(-AutoConstants.N_6, -AutoConstants.N_0_5,
							new Rotation2d(Math.toRadians(-AutoConstants.DEG_90))));
					} else {
						pickUp4.add(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
					}
				}
				return driveAlongPath(pickUp4);
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

		if (input == null) {
			return;
		}
		switch (currentState) {
			case TELEOP_STATE:
				drive(-MathUtil.applyDeadband((input.getControllerLeftJoystickY()
					* Math.abs(input.getControllerLeftJoystickY()) * ((input.getLeftTrigger() / 2)
					+ DriveConstants.LEFT_TRIGGER_DRIVE_CONSTANT) / 2), OIConstants.DRIVE_DEADBAND),
					-MathUtil.applyDeadband((input.getControllerLeftJoystickX()
					* Math.abs(input.getControllerLeftJoystickX()) * ((input.getLeftTrigger() / 2)
					+ DriveConstants.LEFT_TRIGGER_DRIVE_CONSTANT) / 2), OIConstants.DRIVE_DEADBAND),
					-MathUtil.applyDeadband((input.getControllerRightJoystickX()
					* ((input.getLeftTrigger() / 2) + DriveConstants.LEFT_TRIGGER_DRIVE_CONSTANT)
					/ DriveConstants.ANGULAR_SPEED_LIMIT_CONSTANT), OIConstants.DRIVE_DEADBAND),
					true, true);
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
						double z1 = rpi.getAprilTagZInv(VisionConstants.BLUE_SOURCE_TAG1_ID);
						double z2 = rpi.getAprilTagZInv(VisionConstants.BLUE_SOURCE_TAG2_ID);
						if (!(z1 == VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT
							&& z2 == VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT)) {
							if (z1 == VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT) {
								lockedSourceId = VisionConstants.BLUE_SOURCE_TAG2_ID;
							} else if (z2 == VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT) {
								lockedSourceId = VisionConstants.BLUE_SOURCE_TAG1_ID;
							} else {
								if (z1 <= z2) {
									lockedSourceId = VisionConstants.BLUE_SOURCE_TAG1_ID;
								} else {
									lockedSourceId = VisionConstants.BLUE_SOURCE_TAG2_ID;
								}
							}
						}
					} else {
						//id 9 and 10
						double z9 = rpi.getAprilTagZInv(VisionConstants.RED_SOURCE_TAG1_ID);
						double z10 = rpi.getAprilTagZInv(VisionConstants.RED_SOURCE_TAG2_ID);
						if (!(z9 == VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT
							&& z10 == VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT)) {
							if (z9 == VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT) {
								lockedSourceId = VisionConstants.RED_SOURCE_TAG2_ID;
							} else if (z10 == VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT) {
								lockedSourceId = VisionConstants.RED_SOURCE_TAG1_ID;
							} else {
								if (z9 <= z10) {
									lockedSourceId = VisionConstants.RED_SOURCE_TAG1_ID;
								} else {
									lockedSourceId = VisionConstants.RED_SOURCE_TAG2_ID;
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
						if (rpi.getAprilTagZInv(VisionConstants.BLUE_SPEAKER_TAG_ID)
							!= VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT) {
							lockedSpeakerId = VisionConstants.BLUE_SPEAKER_TAG_ID;
						}
					} else {
						//id 4
						if (rpi.getAprilTagZInv(VisionConstants.RED_SPEAKER_TAG_ID)
							!= VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT) {
							lockedSpeakerId = VisionConstants.RED_SPEAKER_TAG_ID;
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
		double y = (blueAlliance ? pose.getY() : -pose.getY());
		double angle = pose.getRotation().getDegrees();

		double xDiff = x - getPose().getX();
		double yDiff = y - getPose().getY();
		double aDiff = angle - getPose().getRotation().getDegrees();

		if (aDiff > AutoConstants.DEG_180) {
			aDiff -= AutoConstants.DEG_360;
		} else if (aDiff < -AutoConstants.DEG_180) {
			aDiff += AutoConstants.DEG_360;
		}

		System.out.println(aDiff);

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
	 * @param id Id of the tag we are positioning towards.
	 * Turns the robot towards the source's april tag and drives
	 * towards it and straightens the robot when driving against the wall. This positions
	 * robot to be at the source to pickup a note.
	 */
	public void alignToSource(int id) {
		double yDiff = rpi.getAprilTagX(id);
		double xDiff = rpi.getAprilTagZ(id) - VisionConstants.SOURCE_TARGET_DISTANCE;
		double aDiff = rpi.getAprilTagXInv(id);

		SmartDashboard.putNumber("x diff", xDiff);
		SmartDashboard.putNumber("y diff", yDiff);
		SmartDashboard.putNumber("a diff", aDiff);

		double xSpeed = clamp(xDiff
			/ VisionConstants.SOURCE_TRANSLATIONAL_ACCEL_CONSTANT,
			-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
			VisionConstants.MAX_SPEED_METERS_PER_SECOND);
		double ySpeed = clamp(yDiff
			/ VisionConstants.SOURCE_TRANSLATIONAL_ACCEL_CONSTANT,
			-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
			VisionConstants.MAX_SPEED_METERS_PER_SECOND);
		double aSpeed = -clamp(aDiff / VisionConstants.SOURCE_ROTATIONAL_ACCEL_CONSTANT,
			-VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
			VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);

		double xSpeedField = Math.abs(xDiff) > VisionConstants.X_MARGIN_TO_SOURCE
			? (xSpeed * Math.cos(Math.toRadians(tagOrientationAngles[id])))
			+ (ySpeed * Math.sin(Math.toRadians(tagOrientationAngles[id]))) : 0;
		double ySpeedField = Math.abs(yDiff) > VisionConstants.X_MARGIN_TO_SOURCE
			? (ySpeed * Math.cos(Math.toRadians(tagOrientationAngles[id])))
			- (xSpeed * Math.sin(Math.toRadians(tagOrientationAngles[id]))) : 0;
		aSpeed = Math.abs(aDiff) > VisionConstants.ROT_MARGIN_TO_SOURCE
			? aSpeed : 0;

		if (!isSourceAligned) {
			drive(xSpeedField, ySpeedField, aSpeed, true, false);
			if (Math.abs(xSpeedField) < VisionConstants.MIN_SPEED_THRESHOLD
				&& Math.abs(ySpeedField) < VisionConstants.MIN_SPEED_THRESHOLD
				&& Math.abs(aSpeed) < VisionConstants.MIN_SPEED_THRESHOLD) {
				isSourceAligned = true;
			}
		} else {
			drive(VisionConstants.SOURCE_DRIVE_FORWARD_POWER, 0, 0, false, false);
		}
	}

	/**
	 * @param id Id of the tag we are positioning towards.
	 * Positions the robot to the correct distance from the speaker to shoot
	 */
	public void alignToSpeaker(int id) {
		double yDiff = rpi.getAprilTagX(id);
		double xDiff = rpi.getAprilTagZ(id) - VisionConstants.SPEAKER_TARGET_DISTANCE;
		double aDiff = rpi.getAprilTagXInv(id);

		double xSpeed = clamp(xDiff
			/ VisionConstants.SPEAKER_TRANSLATIONAL_ACCEL_CONSTANT,
			-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
			VisionConstants.MAX_SPEED_METERS_PER_SECOND);
		double ySpeed = clamp(yDiff
			/ VisionConstants.SPEAKER_TRANSLATIONAL_ACCEL_CONSTANT,
			-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
			VisionConstants.MAX_SPEED_METERS_PER_SECOND);
		double aSpeed = -clamp(aDiff / VisionConstants.SPEAKER_ROTATIONAL_ACCEL_CONSTANT,
			-VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
			VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);

		double xSpeedField = Math.abs(xDiff) > VisionConstants.X_MARGIN_TO_SPEAKER
			? (xSpeed * Math.cos(Math.toRadians(tagOrientationAngles[id])))
			+ (ySpeed * Math.sin(Math.toRadians(tagOrientationAngles[id]))) : 0;
		double ySpeedField = Math.abs(yDiff) > VisionConstants.Y_MARGIN_TO_SPEAKER
			? (ySpeed * Math.cos(Math.toRadians(tagOrientationAngles[id])))
			- (xSpeed * Math.sin(Math.toRadians(tagOrientationAngles[id]))) : 0;
		aSpeed = Math.abs(aDiff) > VisionConstants.ROT_MARGIN_TO_SPEAKER
			? aSpeed : 0;

		drive(xSpeedField, ySpeedField, aSpeed, true, false);
	}

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
