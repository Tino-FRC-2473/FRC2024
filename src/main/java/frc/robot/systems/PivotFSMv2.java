// package frc.robot.systems;

// // WPILib Imports

// // Third party Hardware Imports
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkLimitSwitch;
// //import com.revrobotics.SparkPIDController;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// // Robot Imports
// import frc.robot.TeleopInput;
// import frc.robot.HardwareMap;
// import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

// public class PivotFSMv2 {
// 	/* ======================== Constants ======================== */
// 	// FSM state definitions
// 	public enum PivotFSMState {
// 		ZEROING,
//         IDLE_STOP,
//         SHOOTING,
//         OUTTAKING,
//         MOVE_TO_AMP,
//         FEED_SHOOTER,
//         MOVE_TO_SHOOTER,
//         INTAKING,
//         MOVE_TO_GROUND
// 	}

// 	private static final float MANUAL_POWER = 0.25f;
// 	private static final double MIN_TURN_SPEED = -5.0;
// 	private static final double MAX_TURN_SPEED = 5.0;

// 	private static final double MAX_VELOCITY = 5;
// 	private static final double MAX_ACCEL = 10;

// 	private static final double PID_CONSTANT_PIVOT_P = 0.001;
// 	private static final double PID_CONSTANT_PIVOT_I = 0.001;
// 	private static final double PID_CONSTANT_PIVOT_D = 0.001;


// 	// V = Kg * cos(theta) + Ks * [theta] + Kv * angular vel + Ka * angular acceleration
// 	private static final double PID_VOLTAGE_CONSTANT = 0.4;
// 	private static final double PID_STATIC_CONSTANT = 0.1;
// 	private static final double PID_GRAVITY_CONSTANT = 0.25;


// 	private static final double JOYSTICK_DEAD_ZONE = 0.15;
// 	private static final double MIN_ENCODER_ROTATIONS = -1;
// 	private static final double MAX_ENCODER_ROTATIONS = 80;
// 	private static final double GROUND_ENCODER_ROTATIONS = 50;
// 	private static final double AMP_ENCODER_ROTATIONS = 30;
// 	private static final double SOURCE_ENCODER_ROTATIONS = 25;
// 	private static final double SHOOTER_ENCODER_ROTATIONS = 5;
// 	private static final double INRANGE_VALUE = 0.5;
// 	private static final double JOYSTICK_SCALING_CONSTANT = 0.4;



// 	/* ======================== Private variables ======================== */
// 	private PivotFSMState currentState;
// 	private double currentEncoder = 0;
// 	private double currentTime;
// 	private double lastLoopTime;
// 	private double lastSpeed;
// 	private boolean zeroed = false;
// 	private boolean lastLimitHit = false;
// 	private double pidVal;
// 	private double acceleration;

// 	// Hardware devices should be owned by one and only one system. They must
// 	// be private to their owner system and may not be used elsewhere.
// 	private CANSparkMax pivotMotor;
// 	private CANSparkMax intakeMotor;
// 	private CANSparkMax leftShooterMotor;
// 	private CANSparkMax rightShooterMotor;

// 	private ProfiledPIDController pidPivotController;
// 	//private SparkPIDController pidPivotController;
// 	private SparkLimitSwitch lastLimitSwitch;
// 	private final ArmFeedforward pivotFeedforward;

// 	private boolean hasTimerStarted = false;




// 	/* ======================== Constructor ======================== */
// 	/**
// 	 * Create PivotFSM and initialize to starting state. Also perform any
// 	 * one-time initialization or configuration of hardware required. Note
// 	 * the constructor is called only once when the robot boots.
// 	 */
// 	public PivotFSMv2() {
// 		// Perform hardware init
// 		pivotMotor = new CANSparkMax(HardwareMap.CAN_SPARK_PIVOT_MOTOR,
// 										CANSparkMax.MotorType.kBrushless);

// 		intakeMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_INTAKE_MOTOR,
// 										CANSparkMax.MotorType.kBrushless);

// 		leftShooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_LSHOOTER_MOTOR,
// 										CANSparkMax.MotorType.kBrushless);

// 		rightShooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_RSHOOTER_MOTOR,
// 										CANSparkMax.MotorType.kBrushless);

// 		pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
// 		intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
// 		leftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
// 		rightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);


// 		lastLimitSwitch =
// 		pivotMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

// 		lastLimitSwitch.enableLimitSwitch(false);

// 		/*pidPivotController = pivotMotor.getPIDController();
// 		pidPivotController.setP(PID_CONSTANT_PIVOT_P);
// 		pidPivotController.setI(PID_CONSTANT_PIVOT_I);
// 		pidPivotController.setD(PID_CONSTANT_PIVOT_D);
// 		pidPivotController.setOutputRange(MIN_TURN_SPEED, MAX_TURN_SPEED);*/

// 		pidPivotController = new ProfiledPIDController(PID_CONSTANT_PIVOT_P,
// 			PID_CONSTANT_PIVOT_I, PID_CONSTANT_PIVOT_D,
// 			new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL));

// 		pivotFeedforward = new ArmFeedforward(PID_STATIC_CONSTANT,
// 			PID_GRAVITY_CONSTANT, PID_VOLTAGE_CONSTANT);

// 		pivotMotor.getEncoder().setPosition(0);

// 		currentEncoder = 0;

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

// 		currentTime = Timer.getFPGATimestamp();
// 		lastSpeed = pidPivotController.getSetpoint().velocity;

// 		if (lastLimitSwitch.isPressed()) {
// 			lastLimitHit = true;
// 		}

// 		if (currentState != PivotFSMState.IDLE_STOP) {
// 			currentEncoder = pivotMotor.getEncoder().getPosition();
// 		}

// 		if (input == null) {
// 			return;
// 		}

// 		SmartDashboard.putString("Current State", currentState.toString());
// 		SmartDashboard.putNumber("Motor Power", pivotMotor.get());
// 		SmartDashboard.putNumber("Encoder Value", currentEncoder);
// 		SmartDashboard.putBoolean("Zeroed", zeroed);
// 		SmartDashboard.putBoolean("GROUND BUTTON", input.isGroundArmButtonPressed());
// 		SmartDashboard.putBoolean("SHOOTER BUTTON", input.isShooterArmButtonPressed());
// 		SmartDashboard.putBoolean("SOURCE BUTTON", input.isSourceArmButtonPressed());
// 		SmartDashboard.putBoolean("AMP BUTTON", input.isAmpArmButtonPressed());
// 		SmartDashboard.putBoolean("ABORT BUTTON", input.isAbortButtonPressed());
// 		SmartDashboard.putNumber("PidVAL", pidVal);
// 		SmartDashboard.putNumber("Accel", acceleration);
// 		SmartDashboard.putNumber("Velocity", pidPivotController.getSetpoint().velocity);
// 		SmartDashboard.putNumber(" velocity dif ",
// 			pidPivotController.getSetpoint().velocity - lastSpeed);
// 		SmartDashboard.putNumber("time diff", currentTime - lastLoopTime);
// 		SmartDashboard.putNumber("voltage", pivotMotor.getAppliedOutput());

// 		switch (currentState) {
// 			case IDLE_STOP:
// 				handleIdleState(input);
// 				break;

// 			case ZEROING:
// 				handleZeroingState(input);
// 				break;

// 			case SHOOTING:
// 				handleShootingState(input);
// 				break;

// 			case OUTTAKING_BOTH:
// 				handleOuttakingBothState(input);
// 				break;

// 			case MOVE_TO_AMP:
// 				handleAmpState(input);
// 				break;

// 			case OUTTAKING_INTAKE:
// 				handleOuttakingIntakeState(input);
// 				break;

// 			case MOVE_TO_SHOOTER:
// 				handleShooterState(input);
// 				break;

// 			case INTAKING:
// 				handleIntakingState(input);
// 				break;

// 			case MOVE_TO_GROUND:
// 				handleGroundState(input);
// 				break;

// 			default:
// 				throw new IllegalStateException("Invalid state: " + currentState.toString());
// 		}

// 		lastLoopTime = currentTime;
// 		currentState = nextState(input);
// 	}

// 	/**
// 	 * Performs specific action based on the autoState passed in.
// 	 * @param autoState autoState that the subsystem executes.
// 	 * @return if the action carried out in this state has finished executing
// 	 */
// 	public boolean updateAutonomous(AutoFSMState autoState) {
// 		switch (autoState) {
// 			case STATE1:
// 				return handleAutoState1();
// 			case STATE2:
// 				return handleAutoState2();
// 			case STATE3:
// 				return handleAutoState3();
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
// 				if (!zeroed && !lastLimitHit && !input.isAbortButtonPressed()) {
// 					return PivotFSMState.ZEROING;
// 				} else if (input.isGroundArmButtonPressed()
// 					&& !input.isAbortButtonPressed()
// 					&& !input.isShooterArmButtonPressed()
// 					&& !input.isAmpArmButtonPressed()) {
// 					return PivotFSMState.MOVE_TO_GROUND;
// 				} else if (input.isAmpArmButtonPressed()
// 					&& !input.isAbortButtonPressed()
// 					&& !input.isShooterArmButtonPressed()
// 					&& !input.isGroundArmButtonPressed()
// 					&& !hasNote()) {
// 					return PivotFSMState.MOVE_TO_AMP;
// 				} else if (input.isShooterArmButtonPressed()
// 					&& !input.isAbortButtonPressed()
// 					&& !input.isAmpArmButtonPressed()
// 					&& !input.isGroundArmButtonPressed()) {
// 					return PivotFSMState.MOVE_TO_SHOOTER;
// 				} else {
// 					return PivotFSMState.IDLE_STOP;
// 				}

// 			case ZEROING:
// 				if (!zeroed && !lastLimitHit && !input.isAbortButtonPressed()) {
// 					return PivotFSMState.ZEROING;
// 				} else {
// 					return PivotFSMState.IDLE_STOP;
// 				}

// 			case MOVE_TO_GROUND:
// 				if (!input.isAbortButtonPressed()
// 					&& !inRange(currentEncoder, GROUND_ENCODER_ROTATIONS)) {
// 					return PivotFSMState.MOVE_TO_GROUND;
// 				} else if (input.isAbortButtonPressed()) {
// 					return PivotFSMState.IDLE_STOP;
// 				} else if (!input.isAbortButtonPressed()
// 						&& inRange(currentEncoder, GROUND_ENCODER_ROTATIONS)) {
// 					return PivotFSMState.INTAKING;
// 				}

// 			case INTAKING:
// 				if (!input.isAbortButtonPressed() && !hasNote()) {
// 					return PivotFSMState.INTAKING;
// 				} else if (!input.isAbortButtonPressed() && hasNote()) {
// 					return PivotFSMState.MOVE_TO_SHOOTER;
// 				} else if (input.isAbortButtonPressed()) {
// 					return PivotFSMState.IDLE_STOP;
// 				}

// 			case MOVE_TO_SHOOTER:
// 				if (!input.isAbortButtonPressed()
// 					&& inRange(currentEncoder, SHOOTER_ENCODER_ROTATIONS)) {
// 					return PivotFSMState.MOVE_TO_SHOOTER;
// 				} else {
// 					return PivotFSMState.IDLE_STOP;
// 				}

// 			case MOVE_TO_AMP:
// 				if (!input.isAbortButtonPressed()
// 					&& !inRange(currentEncoder, AMP_ENCODER_ROTATIONS)) {
// 					return PivotFSMState.MOVE_TO_AMP;
// 				} else if (!input.isAbortButtonPressed()
// 					&& inRange(currentEncoder, AMP_ENCODER_ROTATIONS)) {
// 					return PivotFSMState.OUTTAKING;
// 				} else if (input.isAbortButtonPressed()) {
// 					return PivotFSMState.IDLE_STOP;
// 				}

// 			case SOURCE:
// 				if (!input.isAbortButtonPressed()
// 					&& !inRange(currentEncoder, SOURCE_ENCODER_ROTATIONS)) {
// 					return PivotFSMState.SOURCE;
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
// 		pivotMotor.set(0);
// 		lastLoopTime = currentTime;
// 	}
// 	/**
// 	 * Handle behavior in SHOOTER.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	private void handleShooterState(TeleopInput input) {
// 		pidToPosition(SHOOTER_ENCODER_ROTATIONS);
// 	}

// 	/**
// 	 * Handle behavior in AMP.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	private void handleAmpState(TeleopInput input) {
// 		pidToPosition(AMP_ENCODER_ROTATIONS);
// 	}

// 	/**
// 	 * Handle behavior in GROUND.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	private void handleGroundState(TeleopInput input) {
// 		pidToPosition(GROUND_ENCODER_ROTATIONS);
// 	}

// 	/**
// 	 * Handle behavior in SOURCE.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	private void handleSourceState(TeleopInput input) {
// 		pidToPosition(SOURCE_ENCODER_ROTATIONS);
// 	}

// 	/**
// 	 * Handle behavior in ZEROING.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	private void handleZeroingState(TeleopInput input) {
// 		pivotMotor.set(MANUAL_POWER);

// 		if (lastLimitHit && !zeroed) {
// 			zeroed = true;
// 			currentEncoder = 0;
// 			pivotMotor.getEncoder().setPosition(0);
// 		}
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
// 	private boolean handleAutoState1() {
// 		return true;
// 	}

// 	/**
// 	 * Performs action for auto STATE2.
// 	 * @return if the action carried out has finished executing
// 	 */
// 	private boolean handleAutoState2() {
// 		return true;
// 	}

// 	/**
// 	 * Performs action for auto STATE3.
// 	 * @return if the action carried out has finished executing
// 	 */
// 	private boolean handleAutoState3() {
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

// 	/*
//       void updatePos() {
//          double error = goalCount - currentCount;
//          double dt = time - lastTime;
//          errorSum += dt * error;
//          double errorRate = (error - lastError) / dt;
//          double output = (Kp * error) + (Ki * errorSum) + (Kd * errorRate);
// 		 intakeMotor.set(output);
//          lastTime = time;
//          lastError = error;
//  }
//  */

// /**
//  * Limits the motor from spinning faster than the maximum speed.
//  * @param power The power to be applied to the motor
//  * @return Maximum and Minimum power the motor is allowed to run at
//  */
// 	public double clamp(double power) {
// 		return Math.min(MAX_TURN_SPEED, Math.max(MIN_TURN_SPEED, power));
// 	}

// 	/**
// 	 * Calculates the position to PID to and the motor power required.
// 	 * @param goalPosition The position the arm is supposed to go to
// 	 */
// 	public void pidToPosition(double goalPosition) {
// 		pidVal = pidPivotController.calculate(
// 			currentEncoder, goalPosition);
// 		acceleration = (pidPivotController.getSetpoint().velocity
// 			- lastSpeed) / (currentTime - lastLoopTime);

// 		pivotMotor.setVoltage(clamp((pidVal
// 			+ pivotFeedforward.calculate(pidPivotController.getSetpoint().velocity,
// 			acceleration))));
// 	}

// 	private boolean hasNote() {
// 		return true;
// 	}
// }
