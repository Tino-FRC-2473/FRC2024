package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
//import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

public class PivotFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum PivotFSMState {
		IDLE_STOP,
		MANUAL_IN,
		MANUAL_OUT,
		SHOOTING,
		SOURCE,
		AMP,
		GROUND,
		ZEROING
	}

	private static final float INTAKE_POWER = 0.1f;
	private static final double MIN_TURN_SPEED = -0.2;
	private static final double MAX_TURN_SPEED = 0.2;

	private static final double MAX_VELOCITY = 5;
	private static final double MAX_ACCEL = 10;

	private static final double PID_CONSTANT_PIVOT_P = 0.001;
	private static final double PID_CONSTANT_PIVOT_I = 0.001;
	private static final double PID_CONSTANT_PIVOT_D = 0.001;
	private static final double PID_VOLTAGE_CONSTANT = 0;
	private static final double PID_STATIC_CONSTANT = 0;
	private static final double PID_GRAVITY_CONSTANT = 0;


	private static final double JOYSTICK_DEAD_ZONE = 0.15;
	private static final double MIN_ENCODER_ROTATIONS = -100;
	private static final double MAX_ENCODER_ROTATIONS = 100;
	private static final double GROUND_ENCODER_ROTATIONS = 10;
	private static final double AMP_ENCODER_ROTATIONS = 20;
	private static final double SOURCE_ENCODER_ROTATIONS = 30;
	private static final double SHOOTER_ENCODER_ROTATIONS = 0;


	/* ======================== Private variables ======================== */
	private PivotFSMState currentState;
	private double currentEncoder = 0;
	private double currentTime;
	private double lastLoopTime;
	private double lastSpeed;

	private boolean zeroed = false;
	private boolean lastLimitHit = false;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax pivotMotor;

	private ProfiledPIDController pidPivotController;
	//private SparkPIDController pidPivotController;
	private SparkLimitSwitch lastLimitSwitch;
	private SparkLimitSwitch hasNoteSwitch;
	private final ArmFeedforward pivotFeedforward;

	private boolean hasTimerStarted = false;





	/* ======================== Constructor ======================== */
	/**
	 * Create PivotFSM and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public PivotFSM() {
		// Perform hardware init
		pivotMotor = new CANSparkMax(HardwareMap.CAN_SPARK_PIVOT_MOTOR,
										CANSparkMax.MotorType.kBrushless);

		pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		lastLimitSwitch =
		pivotMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

		lastLimitSwitch.enableLimitSwitch(false);

		hasNoteSwitch =
		pivotMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

		hasNoteSwitch.enableLimitSwitch(false);

		/*pidPivotController = pivotMotor.getPIDController();
		pidPivotController.setP(PID_CONSTANT_PIVOT_P);
		pidPivotController.setI(PID_CONSTANT_PIVOT_I);
		pidPivotController.setD(PID_CONSTANT_PIVOT_D);
		pidPivotController.setOutputRange(MIN_TURN_SPEED, MAX_TURN_SPEED);*/

		pidPivotController = new ProfiledPIDController(PID_CONSTANT_PIVOT_P,
			PID_CONSTANT_PIVOT_I, PID_CONSTANT_PIVOT_D,
			new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL));

		pivotFeedforward = new ArmFeedforward(PID_STATIC_CONSTANT, PID_GRAVITY_CONSTANT, PID_VOLTAGE_CONSTANT);

		pivotMotor.getEncoder().setPosition(0);

		currentEncoder = 0;

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public PivotFSMState getCurrentState() {
		return currentState;
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
		currentState = PivotFSMState.IDLE_STOP;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {

		if (input == null) {
			return;
		}

		if (lastLimitSwitch.isPressed()) {
			lastLimitHit = true;
		}

		if (currentState != PivotFSMState.IDLE_STOP) {
			currentEncoder = pivotMotor.getEncoder().getPosition();
		}

		if (currentTime != 0) {
			lastLoopTime = currentTime;
		}

		switch (currentState) {
			case IDLE_STOP:
				handleIdleState(input);
				break;

			case SHOOTING:
				handleShootingState(input);
				break;

			case ZEROING:
				handleZeroingState(input);
				break;

			case MANUAL_IN:
				handleManualState(input, true);
				break;

			case MANUAL_OUT:
				handleManualState(input, false);
				break;

			case GROUND:
				handleGroundState(input);
				break;

			case AMP:
				handleAmpState(input);
				break;

			case SOURCE:
				handleSourceState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);
		currentTime = Timer.getFPGATimestamp();
	}

	/**
	 * Performs specific action based on the autoState passed in.
	 * @param autoState autoState that the subsystem executes.
	 * @return if the action carried out in this state has finished executing
	 */
	public boolean updateAutonomous(AutoFSMState autoState) {
		switch (autoState) {
			case STATE1:
				return handleAutoState1();
			case STATE2:
				return handleAutoState2();
			case STATE3:
				return handleAutoState3();
			default:
				return true;
		}
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
	private PivotFSMState nextState(TeleopInput input) {
		if (input == null) {
			return PivotFSMState.IDLE_STOP;
		}
		switch (currentState) {
			case IDLE_STOP:
				if (!zeroed && !lastLimitHit) {
					return PivotFSMState.ZEROING;
				} else if (currentEncoder > MIN_ENCODER_ROTATIONS
					&& input.getLeftJoystickY() < -JOYSTICK_DEAD_ZONE) {
					return PivotFSMState.MANUAL_IN;
				} else if (currentEncoder <= MAX_ENCODER_ROTATIONS
					&& input.getLeftJoystickY() > JOYSTICK_DEAD_ZONE) {
					return PivotFSMState.MANUAL_OUT;
				} else if (input.isGroundButtonPressed()
					&& !inRange(currentEncoder, GROUND_ENCODER_ROTATIONS)) {
					return PivotFSMState.GROUND;
				} else if (input.isAmpButtonPressed()
					&& !inRange(currentEncoder, AMP_ENCODER_ROTATIONS)) {
					return PivotFSMState.AMP;
				} else if (input.isSourceButtonPressed()
					&& !inRange(currentEncoder, SOURCE_ENCODER_ROTATIONS)) {
					return PivotFSMState.SOURCE;
				} else if (input.isShooterButtonPressed()
					&& !inRange(currentEncoder, SHOOTER_ENCODER_ROTATIONS)) {
					return PivotFSMState.SHOOTING;
				}

			case ZEROING:
				if (!zeroed && !lastLimitHit && !input.isAbortButtonPressed()) {
					return PivotFSMState.ZEROING;
				} else {
					return PivotFSMState.IDLE_STOP;
				}
			
			case GROUND:
				if(!input.isAbortButtonPressed() && !inRange(currentEncoder, GROUND_ENCODER_ROTATIONS)){
					return PivotFSMState.GROUND;
				} else {
					return PivotFSMState.IDLE_STOP;
				}
			
			case AMP:
				if(!input.isAbortButtonPressed() && inRange(currentEncoder, AMP_ENCODER_ROTATIONS)){
					return PivotFSMState.AMP;
				} else {
					return PivotFSMState.IDLE_STOP;
				}

			case SOURCE:
				if(!input.isAbortButtonPressed() && inRange(currentEncoder, SOURCE_ENCODER_ROTATIONS)){
					return PivotFSMState.SOURCE;
				} else {
					return PivotFSMState.IDLE_STOP;
				}

			case SHOOTING:
				if(!input.isAbortButtonPressed() && inRange(currentEncoder, SHOOTER_ENCODER_ROTATIONS)){
					return PivotFSMState.SHOOTING;
				} else {
					return PivotFSMState.IDLE_STOP;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in IDLE_STOP.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		pivotMotor.set(0);
	}
	/**
	 * Handle behavior in INTAKING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakingState(TeleopInput input) {
		pivotMotor.set(INTAKE_POWER);
	}

	/**
	 * Handle behavior in OUTTAKING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakingState(TeleopInput input) {
		pivotMotor.set(-INTAKE_POWER); // DO NOT FORGET THE - SIGN
	}

	/**
	 * Performs action for auto STATE1.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState1() {
		return true;
	}

	/**
	 * Performs action for auto STATE2.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState2() {
		return true;
	}

	/**
	 * Performs action for auto STATE3.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState3() {
		return true;
	}

	private boolean inRange(double a, double b) {
		return Math.abs(a - b) > 0.5; //EXPERIMENTAL
	}

	public void pidToPosition(double goalPosition) {
		double pidVal = pidPivotController.calculate(
			pivotMotor.getEncoder().getPosition(), goalPosition);
		double acceleration = (pidPivotController.getSetpoint().velocity
			- lastSpeed) / (currentTime - lastLoopTime);
	
		pivotMotor.setVoltage(pidVal + pivotFeedforward.calculate(pidPivotController.getSetpoint().velocity, acceleration));

		lastSpeed = pidPivotController.getSetpoint().velocity;
	}
}
