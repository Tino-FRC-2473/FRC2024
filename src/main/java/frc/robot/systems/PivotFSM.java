// package frc.robot.systems;


// // WPILib Imports

// // Third party Hardware Imports
// //import com.revrobotics.SparkPIDController;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// // Robot Imports
// import frc.robot.TeleopInput;
// import frc.robot.HardwareMap;
// import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

// public class PivotFSM {
// 	/* ======================== Constants ======================== */
// 	// FSM state definitions
// 	public enum PivotFSMState {
// 		IDLE_STOP,
// 		MANUAL_IN,
// 		MANUAL_OUT,
// 		SHOOTER,
// 		AMP,
// 		GROUND
// 	}


// 	private static final double MIN_TURN_SPEED = -0.15;
// 	private static final double MAX_TURN_SPEED = 0.15;

// 	private static final double PID_CONSTANT_PIVOT_P = 0.001;
// 	private static final double PID_CONSTANT_PIVOT_I = 0.001;
// 	private static final double PID_CONSTANT_PIVOT_D = 0.001;

// 	private static final double JOYSTICK_DEAD_ZONE = 0.05;
// 	private static final double MIN_ENCODER_ROTATIONS = -1000;
// 	private static final double MAX_ENCODER_ROTATIONS = 0;
// 	private static final double GROUND_ENCODER_ROTATIONS = -1000;
// 	private static final double AMP_ENCODER_ROTATIONS = -300;
// 	private static final double SOURCE_ENCODER_ROTATIONS = -100;
// 	private static final double SHOOTER_ENCODER_ROTATIONS = 0;
// 	private static final double INRANGE_VALUE = 5;
// 	private static final double JOYSTICK_SCALING_CONSTANT = 0.15;



// 	/* ======================== Private variables ======================== */
// 	private PivotFSMState currentState;
// 	private double currentEncoder = 0;

// 	// Hardware devices should be owned by one and only one system. They must
// 	// be private to their owner system and may not be used elsewhere.
// 	private TalonFX pivotMotor;

// 	private DigitalInput lastLimitSwitch;

// 	private Encoder throughBore;


// 	/* ======================== Constructor ======================== */
// 	/**
// 	 * Create PivotFSM and initialize to starting state. Also perform any
// 	 * one-time initialization or configuration of hardware required. Note
// 	 * the constructor is called only once when the robot boots.
// 	 */
// 	public PivotFSM() {
// 		// Perform hardware init
// 		// pivotMotor = new CANSparkMax(HardwareMap.CAN_SPARK_PIVOT_MOTOR,
// 		// 								CANSparkMax.MotorType.kBrushless);

// 		// pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

// 		pivotMotor = new TalonFX(HardwareMap.DEVICE_ID_ARM_MOTOR);
// 		pivotMotor.setNeutralMode(NeutralModeValue.Brake);
// 		lastLimitSwitch = new DigitalInput(HardwareMap.INPUT_LIMIT_SWITCH_PORT);
// 		currentEncoder = 0;
// 		throughBore = new Encoder(0, 1);
// 		throughBore.reset();


// 		// Reset state machine
// 		reset();
// 	}

// 	/* ======================== Public methods ======================== */
// 	/**
// 	 * Return current FSM state.
// 	 * @return Current FSM state
// 	 */
// 	public PivotFSMState getCurrentState() {
// 		return currentState;
// 	}
// 	/**
// 	 * Reset this system to its start state. This may be called from mode init
// 	 * when the robot is enabled.
// 	 *
// 	 * Note this is distinct from the one-time initialization in the constructor
// 	 * as it may be called multiple times in a boot cycle,
// 	 * Ex. if the robot is enabled, disabled, then reenabled.
// 	 */
// 	public void reset() {
// 		currentState = PivotFSMState.IDLE_STOP;

// 		// Call one tick of update to ensure outputs reflect start state
// 		update(null);
// 	}

// 	/**
// 	 * Update FSM based on new inputs. This function only calls the FSM state
// 	 * specific handlers.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	public void update(TeleopInput input) {

// 		// if (lastLimitSwitch.get()) {
// 		// 	currentEncoder = 0;
// 		// 	throughBore.reset();
// 		// }

// 		if (currentState != PivotFSMState.IDLE_STOP) {
// 			currentEncoder = throughBore.getDistance();
// 		}

// 		if (input == null) {
// 			return;
// 		}

// 		SmartDashboard.putString("Current State", currentState.toString());
// 		SmartDashboard.putNumber("Motor Power", pivotMotor.get());
// 		//SmartDashboard.putNumber("Encoder Value", currentEncoder);
// 		//SmartDashboard.putNumber("PidVAL", pidVal);
// 		//SmartDashboard.putNumber("Accel", acceleration);
// 		//SmartDashboard.putNumber("Velocity", pidPivotController.getSetpoint().velocity);
// 		//SmartDashboard.putNumber("voltage", pivotMotor.getAppliedOutput());
// 		SmartDashboard.putNumber("Thru Bore Encoder values", throughBore.getDistance());
// 		SmartDashboard.putNumber("CURRENT ENCODER", currentEncoder);
// 		SmartDashboard.putBoolean("Bottom Limit Switch", lastLimitSwitch.get());
// 		SmartDashboard.putBoolean("Ground pivot burron", input.isGroundArmButtonPressed());
// 		switch (currentState) {
// 			case IDLE_STOP:
// 				handleIdleState(input);
// 				break;

// 			case MANUAL_IN:
// 				handleManualInState(input);
// 				break;

// 			case MANUAL_OUT:
// 				handleManualOutState(input);
// 				break;

// 			case SHOOTER:
// 				handleShooterState(input);
// 				break;

// 			case GROUND:
// 				handleGroundState(input);
// 				break;

// 			case AMP:
// 				handleAmpState(input);
// 				break;

// 			default:
// 				throw new IllegalStateException("Invalid state: " + currentState.toString());
// 		}

// 		currentState = nextState(input);
// 	}

// 	/**
// 	 * Performs specific action based on the autoState passed in.
// 	 * @param autoState autoState that the subsystem executes.
// 	 * @return if the action carried out in this state has finished executing
// 	 */
// 	public boolean updateAutonomous(AutoFSMState autoState) {
// 		// if (lastLimitSwitch.get()) {
// 		// 	currentEncoder = 0;
// 		// 	throughBore.reset();
// 		// }

// 		if (autoState != AutoFSMState.SHOOT) {
// 			currentEncoder = throughBore.getDistance();
// 		}
// 		switch (autoState) {
// 			case DRIVE_TO_NOTE:
// 				return handleAutoMoveToGround();
// 			case DRIVE_TO_SPEAKER:
// 				return handleAutoMoveToShooter();
// 			case SHOOT:
// 				return handleAutoIdle();
// 			default:
// 				return true;
// 		}
// 	}

// 	/* ======================== Private methods ======================== */
// 	/**
// 	 * Decide the next state to transition to. This is a function of the inputs
// 	 * and the current state of this FSM. This method should not have any side
// 	 * effects on outputs. In other words, this method should only read or get
// 	 * values to decide what state to go to.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 * @return FSM state for the next iteration
// 	 */
// 	private PivotFSMState nextState(TeleopInput input) {
// 		if (input == null) {
// 			return PivotFSMState.IDLE_STOP;
// 		}
// 		switch (currentState) {
// 			case IDLE_STOP:
// 				if (currentEncoder > MIN_ENCODER_ROTATIONS
// 					&& input.getMechControllerLeftY() <= -JOYSTICK_DEAD_ZONE) {
// 					return PivotFSMState.MANUAL_IN;
// 				} else if (currentEncoder <= MAX_ENCODER_ROTATIONS
// 					&& input.getMechControllerLeftY() >= JOYSTICK_DEAD_ZONE) {
// 					return PivotFSMState.MANUAL_OUT;
// 				} else if (input.isGroundArmButtonPressed()
// 					&& !inRange(currentEncoder, GROUND_ENCODER_ROTATIONS)) {
// 					return PivotFSMState.GROUND;
// 				} else if (input.isAmpArmButtonPressed()
// 					&& !inRange(currentEncoder, AMP_ENCODER_ROTATIONS)) {
// 					return PivotFSMState.AMP;
// 				} else if (input.isShooterArmButtonPressed()
// 					&& !inRange(currentEncoder, SHOOTER_ENCODER_ROTATIONS)) {
// 					return PivotFSMState.SHOOTER;
// 				} else {
// 					return PivotFSMState.IDLE_STOP;
// 				}

// 			case MANUAL_OUT:
// 				if (currentEncoder <= MAX_ENCODER_ROTATIONS
// 					&& input.getMechControllerLeftY() >= JOYSTICK_DEAD_ZONE) {
// 					return PivotFSMState.MANUAL_OUT;
// 				}
// 				return PivotFSMState.IDLE_STOP;

// 			case MANUAL_IN:
// 				if (currentEncoder > MIN_ENCODER_ROTATIONS
// 					&& input.getMechControllerLeftY() <= -JOYSTICK_DEAD_ZONE) {
// 					return PivotFSMState.MANUAL_IN;
// 				}
// 				return PivotFSMState.IDLE_STOP;

// 			case GROUND:
// 				if (!input.isAbortButtonPressed()
// 					&& !inRange(currentEncoder, GROUND_ENCODER_ROTATIONS)) {
// 					return PivotFSMState.GROUND;
// 				} else {
// 					return PivotFSMState.IDLE_STOP;
// 				}

// 			case AMP:
// 				if (!input.isAbortButtonPressed()
// 					&& !inRange(currentEncoder, AMP_ENCODER_ROTATIONS)) {
// 					return PivotFSMState.AMP;
// 				} else {
// 					return PivotFSMState.IDLE_STOP;
// 				}

// 			case SHOOTER:
// 				if (!input.isAbortButtonPressed()
// 					&& !inRange(currentEncoder, SHOOTER_ENCODER_ROTATIONS)) {
// 					return PivotFSMState.SHOOTER;
// 				} else {
// 					return PivotFSMState.IDLE_STOP;
// 				}

// 			default:
// 				throw new IllegalStateException("Invalid state: " + currentState.toString());
// 		}
// 	}

// 	/* ------------------------ FSM state handlers ------------------------ */
// 	/**
// 	 * Handle behavior in IDLE_STOP.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	private void handleIdleState(TeleopInput input) {
// 		pivotMotor.set(pid(throughBore.getDistance(), currentEncoder));
// 	}
// 	/**
// 	 * Handle behavior in SHOOTER.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	private void handleShooterState(TeleopInput input) {
// 		pivotMotor.set(pid(currentEncoder, SHOOTER_ENCODER_ROTATIONS));
// 	}

// 	/**
// 	 * Handle behavior in AMP.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	private void handleAmpState(TeleopInput input) {
// 		pivotMotor.set(pid(currentEncoder, AMP_ENCODER_ROTATIONS));
// 	}

// 	/**
// 	 * Handle behavior in GROUND.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	private void handleGroundState(TeleopInput input) {
// 		pivotMotor.set(pid(currentEncoder, GROUND_ENCODER_ROTATIONS));
// 	}



// 	/**
// 	 * Handle behavior in MANUAL_IN.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	public void handleManualInState(TeleopInput input) {
// 		pivotMotor.set(input.getMechControllerLeftY() * JOYSTICK_SCALING_CONSTANT);
// 	}

// 	/**
// 	 * Handle behavior in MANUAL_OUT.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	public void handleManualOutState(TeleopInput input) {
// 		pivotMotor.set(input.getMechControllerLeftY() * JOYSTICK_SCALING_CONSTANT);
// 	}

// 	/**
// 	 * Performs action for auto STATE1.
// 	 * @return if the action carried out has finished executing
// 	 */
// 	private boolean handleAutoMoveToGround() {
// 		pivotMotor.set(pid(currentEncoder, GROUND_ENCODER_ROTATIONS));
// 		return inRange(currentEncoder, GROUND_ENCODER_ROTATIONS);
// 	}

// 	/**
// 	 * Performs action for auto STATE2.
// 	 * @return if the action carried out has finished executing
// 	 */
// 	private boolean handleAutoMoveToShooter() {
// 		pivotMotor.set(pid(currentEncoder, SHOOTER_ENCODER_ROTATIONS));
// 		return inRange(currentEncoder, SHOOTER_ENCODER_ROTATIONS);
// 	}

// 	/**
// 	 * Performs action for auto STATE3.
// 	 * @return if the action carried out has finished executing
// 	 */
// 	private boolean handleAutoIdle() {
// 		pivotMotor.set(pid(throughBore.getDistance(), currentEncoder));
// 		return true;
// 	}

// 	/**
// 	 * Stops the motor when Current Encoder Value is near the Target Encoder Value.
// 	 * @param a Current Encoder Value
// 	 * @param b Target Encoder Value
// 	 * @return if the distnace between Current Encoder Value and Target Encoder Value
// 	 *  is less than INRANGE_VALUE
// 	 */
// 	private boolean inRange(double a, double b) {
// 		return Math.abs(a - b) < INRANGE_VALUE; //EXPERIMENTAL
// 	}

// 	/**
// 	 * Limits the motor from spinning faster than the maximum speed.
// 	 * @param power The power to be applied to the motor
// 	 * @return Maximum and Minimum power the motor is allowed to run at
// 	 */
// 	public double clamp(double power) {
// 		return Math.min(MAX_TURN_SPEED, Math.max(MIN_TURN_SPEED, power));
// 	}


// 	private double pid(double currentEncoderPID, double targetEncoder) {
// 		double correction = PID_CONSTANT_PIVOT_P * (targetEncoder - currentEncoderPID);
// 		return Math.min(MAX_TURN_SPEED, Math.max(MIN_TURN_SPEED, correction));
// 	}
// }
