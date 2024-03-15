package frc.robot.systems;


// WPILib Imports

// Third party Hardware Imports

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

public class PivotFSMv2 {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum PivotFSMState {
		ZEROING,
		IDLE_STOP,
		MOVE_TO_GROUND,
		INTAKING,
		MOVE_TO_SHOOTER,
		OUTTAKING,
		MOVE_TO_AMP,
		REV_UP_SHOOTER,
		FEED_SHOOTER
	}


	private static final double MIN_TURN_SPEED = -0.15;
	private static final double MAX_TURN_SPEED = 0.15;

	private static final float INTAKE_POWER = 0.1f;
	private static final float OUTTAKE_POWER = -0.1f;
	private static final float HOLDING_POWER = 0.03f;
	private static final float SHOOTING_POWER = 0.7f;

	private static final double PID_CONSTANT_PIVOT_P = 0.001;
	private static final double PID_CONSTANT_PIVOT_I = 0.001;
	private static final double PID_CONSTANT_PIVOT_D = 0.001;

	private static final double JOYSTICK_DEAD_ZONE = 0.05;
	private static final double MIN_ENCODER_ROTATIONS = -1000;
	private static final double MAX_ENCODER_ROTATIONS = 0;
	private static final double GROUND_ENCODER_ROTATIONS = -1000;
	private static final double AMP_ENCODER_ROTATIONS = -300;
	private static final double SOURCE_ENCODER_ROTATIONS = -100;
	private static final double SHOOTER_ENCODER_ROTATIONS = 0;
	private static final double INRANGE_VALUE = 5;
	private static final double JOYSTICK_SCALING_CONSTANT = 0.15;



	/* ======================== Private variables ======================== */
	private PivotFSMState currentState;
	private double currentEncoder = 0;
	private double currentTime;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private TalonFX pivotMotor;
	private TalonFX intakeMotor;
	private CANSparkMax leftShooterMotor;
	private CANSparkMax rightShooterMotor;

	private DigitalInput lastLimitSwitch;

	private boolean hasTimerStarted = false;

	private Encoder throughBore;



	/* ======================== Constructor ======================== */
	/**
	 * Create PivotFSM and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public PivotFSMv2() {
		// Perform hardware init
		pivotMotor = new TalonFX(0);

		intakeMotor = new TalonFX(1);

		leftShooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_LSHOOTER_MOTOR,
										CANSparkMax.MotorType.kBrushless);

		rightShooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_RSHOOTER_MOTOR,
										CANSparkMax.MotorType.kBrushless);

		pivotMotor.setNeutralMode(NeutralModeValue.Brake);
		intakeMotor.setNeutralMode(NeutralModeValue.Brake);
		leftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);


		lastLimitSwitch = new DigitalInput(HardwareMap.INPUT_LIMIT_SWITCH_PORT);

		/*pidPivotController = pivotMotor.getPIDController();
		pidPivotController.setP(PID_CONSTANT_PIVOT_P);
		pidPivotController.setI(PID_CONSTANT_PIVOT_I);
		pidPivotController.setD(PID_CONSTANT_PIVOT_D);
		pidPivotController.setOutputRange(MIN_TURN_SPEED, MAX_TURN_SPEED);*/

		currentEncoder = 0;
		throughBore = new Encoder(0, 1);
		throughBore.reset();

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


		if (currentState != PivotFSMState.IDLE_STOP) {
			currentEncoder = throughBore.getDistance();
		}

		if (input == null) {
			return;
		}

		SmartDashboard.putString("Current State", currentState.toString());
		SmartDashboard.putNumber("Motor Power", pivotMotor.get());
		SmartDashboard.putNumber("Encoder Value", currentEncoder);
		SmartDashboard.putBoolean("GROUND BUTTON", input.isIntakeButtonPressed());
		SmartDashboard.putBoolean("SHOOTER BUTTON", input.isShootButtonPressed());
		SmartDashboard.putBoolean("REV UP BUTTON", input.isRevUpButtonPressed());
		SmartDashboard.putBoolean("AMP BUTTON", input.isAmpButtonPressed());
		SmartDashboard.putBoolean("ABORT BUTTON", input.isAbortButtonPressed());

		switch (currentState) {
			case IDLE_STOP:
				handleIdleState(input);
				break;

			case FEED_SHOOTER:
				handleFeedShooterState(input);
				break;

			case REV_UP_SHOOTER:
				handleRevUpState(input);
				break;

			case MOVE_TO_AMP:
				handleAmpState(input);
				break;

			case OUTTAKING:
				handleOuttakingState(input);
				break;

			case MOVE_TO_SHOOTER:
				handleShooterState(input);
				break;

			case INTAKING:
				handleIntakingState(input);
				break;

			case MOVE_TO_GROUND:
				handleGroundState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/**
	 * Performs specific action based on the autoState passed in.
	 * @param autoState autoState that the subsystem executes.
	 * @return if the action carried out in this state has finished executing
	 */
	public boolean updateAutonomous(AutoFSMState autoState) {
		switch (autoState) {
			case DRIVE_TO_NOTE:
				return handleAutoState1();
			case DRIVE_TO_SPEAKER:
				return handleAutoState2();
			case SHOOT:
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
				if (input.isAmpButtonPressed()
					&& !input.isAbortButtonPressed()
					&& !input.isShootButtonPressed()
					&& !input.isIntakeButtonPressed()
					&& !input.isRevUpButtonPressed()) {
					return PivotFSMState.MOVE_TO_GROUND;
				} else if (input.isAmpButtonPressed()
					&& !input.isAbortButtonPressed()
					&& !input.isShootButtonPressed()
					&& !input.isIntakeButtonPressed()
					&& !input.isRevUpButtonPressed()) {
					return PivotFSMState.MOVE_TO_AMP;
				} else if (input.isRevUpButtonPressed()
					&& !input.isAbortButtonPressed()
					&& !input.isIntakeButtonPressed()
					&& !input.isAmpButtonPressed()) {
					return PivotFSMState.MOVE_TO_SHOOTER;
				} else {
					return PivotFSMState.IDLE_STOP;
				}

			case MOVE_TO_GROUND:
				if (!input.isAbortButtonPressed()
					&& !inRange(currentEncoder, GROUND_ENCODER_ROTATIONS)
					&& input.isIntakeButtonPressed()) {
					return PivotFSMState.MOVE_TO_GROUND;
				} else if (input.isAbortButtonPressed()) {
					return PivotFSMState.IDLE_STOP;
				} else if (!input.isAbortButtonPressed()
						&& inRange(currentEncoder, GROUND_ENCODER_ROTATIONS)
						&& input.isIntakeButtonPressed()) {
					return PivotFSMState.INTAKING;
				}

			case INTAKING:
				if (!input.isAbortButtonPressed() && !hasNote()
					&& input.isIntakeButtonPressed()) {
					return PivotFSMState.INTAKING;
				} else if ((!input.isAbortButtonPressed() && hasNote())
							|| !input.isIntakeButtonPressed()) {
					return PivotFSMState.MOVE_TO_SHOOTER;
				} else if (input.isAbortButtonPressed()) {
					return PivotFSMState.IDLE_STOP;
				}

			case MOVE_TO_SHOOTER:
				if (!input.isAbortButtonPressed()
					&& inRange(currentEncoder, SHOOTER_ENCODER_ROTATIONS)) {
					return PivotFSMState.MOVE_TO_SHOOTER;
				} else if (input.isAbortButtonPressed()
					|| inRange(currentEncoder, SHOOTER_ENCODER_ROTATIONS)
					&& !input.isRevUpButtonPressed()) {
					return PivotFSMState.IDLE_STOP;
				} else if (!input.isAbortButtonPressed()
					&& inRange(currentEncoder, SHOOTER_ENCODER_ROTATIONS)
					&& input.isRevUpButtonPressed()) {
					return PivotFSMState.REV_UP_SHOOTER;
				}

			case REV_UP_SHOOTER:
				if (input.isRevUpButtonPressed()
					&& !input.isShootButtonPressed()
					&& !input.isAbortButtonPressed()) {
					return PivotFSMState.REV_UP_SHOOTER;
				} else if (!input.isAbortButtonPressed()
					&& input.isRevUpButtonPressed()
					&& input.isShootButtonPressed()) {
					return PivotFSMState.FEED_SHOOTER;
				} else if (input.isAbortButtonPressed()
					|| !input.isRevUpButtonPressed()) {
					return PivotFSMState.IDLE_STOP;
				}

			case MOVE_TO_AMP:
				if (!input.isAbortButtonPressed()
					&& !inRange(currentEncoder, AMP_ENCODER_ROTATIONS)
					&& input.isAmpButtonPressed()) {
					return PivotFSMState.MOVE_TO_AMP;
				} else if (!input.isAbortButtonPressed()
					&& inRange(currentEncoder, AMP_ENCODER_ROTATIONS)
					&& input.isAmpButtonPressed()) {
					return PivotFSMState.OUTTAKING;
				} else if (input.isAbortButtonPressed()) {
					return PivotFSMState.IDLE_STOP;
				}

			case OUTTAKING:
				if (!input.isAbortButtonPressed()
					&& input.isAmpButtonPressed()
					&& hasNote()) {
					return PivotFSMState.OUTTAKING;
				} else if ((!input.isAbortButtonPressed() || !hasNote()
					|| !input.isAmpButtonPressed())) {
					return PivotFSMState.MOVE_TO_SHOOTER;
				} else if (input.isAbortButtonPressed()) {
					return PivotFSMState.IDLE_STOP;
				}

			case FEED_SHOOTER:
				if (input.isAbortButtonPressed()
					|| !input.isRevUpButtonPressed()
					&& !input.isShootButtonPressed()) {
					return PivotFSMState.IDLE_STOP;
				} else if (!input.isAbortButtonPressed()
					&& input.isRevUpButtonPressed()
					&& input.isShootButtonPressed()) {
					return PivotFSMState.FEED_SHOOTER;
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
		pivotMotor.set(pid(currentEncoder, throughBore.getDistance()));
	}
	/**
	 * Handle behavior in SHOOTER.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleShooterState(TeleopInput input) {
		pivotMotor.set(pid(currentEncoder, SHOOTER_ENCODER_ROTATIONS));
	}

	/**
	 * Handle behavior in AMP.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleAmpState(TeleopInput input) {
		pivotMotor.set(pid(currentEncoder, AMP_ENCODER_ROTATIONS));
	}

	/**
	 * Handle behavior in GROUND.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleGroundState(TeleopInput input) {
		pivotMotor.set(pid(currentEncoder, GROUND_ENCODER_ROTATIONS));
	}

	/**
	 * Handle behavior in MANUAL_IN.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleManualInState(TeleopInput input) {
		pivotMotor.set(input.getMechControllerLeftY() * JOYSTICK_SCALING_CONSTANT);
	}

	/**
	 * Handle behavior in MANUAL_OUT.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleManualOutState(TeleopInput input) {
		pivotMotor.set(input.getMechControllerLeftY() * JOYSTICK_SCALING_CONSTANT);
	}

	/**
	 * Handle behavior in FEED_SHOOTER.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleFeedShooterState(TeleopInput input) {
		intakeMotor.set(OUTTAKE_POWER);
		leftShooterMotor.set(SHOOTING_POWER);
		rightShooterMotor.set(-SHOOTING_POWER);
	}

	/**
	 * Handle behavior in REV_UP_SHOOTER.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleRevUpState(TeleopInput input) {
		leftShooterMotor.set(SHOOTING_POWER);
		rightShooterMotor.set(-SHOOTING_POWER);
	}

	/**
	 * Handle behavior in OUTTAKING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleOuttakingState(TeleopInput input) {
		intakeMotor.set(OUTTAKE_POWER);
	}

	/**
	 * Handle behavior in INTAKING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleIntakingState(TeleopInput input) {
		intakeMotor.set(INTAKE_POWER);
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

	/**
	 * Stops the motor when Current Encoder Value is near the Target Encoder Value.
	 * @param a Current Encoder Value
	 * @param b Target Encoder Value
	 * @return if the distnace between Current Encoder Value and Target Encoder Value
	 *  is less than INRANGE_VALUE
	 */
	private boolean inRange(double a, double b) {
		return Math.abs(a - b) < INRANGE_VALUE; //EXPERIMENTAL
	}

/**
 * Limits the motor from spinning faster than the maximum speed.
 * @param power The power to be applied to the motor
 * @return Maximum and Minimum power the motor is allowed to run at
 */
	public double clamp(double power) {
		return Math.min(MAX_TURN_SPEED, Math.max(MIN_TURN_SPEED, power));
	}

	/**
	 * Calculates the position to PID to and the motor power required.
	 * @param currentEncoderPID The position the arm is currently at
	 * @param targetEncoder The position the arm is supposed to go to
	 */
	private double pid(double currentEncoderPID, double targetEncoder) {
		double correction = PID_CONSTANT_PIVOT_P * (targetEncoder - currentEncoderPID);
		return -Math.min(MAX_TURN_SPEED, Math.max(MIN_TURN_SPEED, correction));
	}

	private boolean hasNote() {
		return true;
	}
}
