package frc.robot.systems;

// WPILib Imports

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.HardwareMap;

// Third party Hardware Imports

// Robot Imports
import frc.robot.TeleopInput;

public class KrakenTestFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum KrakenTestFSMState {
		IDLE_STOP,
		RUN_MOTOR,
		RUN_HALF,
		RUN_FULL
	}

	private static final float RUN_SPEED = 0.1f;
	private static final float HALF_SPEED = 0.5f;
	private static final float FULL_SPEED = 1.0f;

	/* ======================== Private variables ======================== */
	private KrakenTestFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private TalonFX testMotor;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public KrakenTestFSM() {
		// Perform hardware init
		testMotor = new TalonFX(HardwareMap.KRAKEN_ID);
		testMotor.setNeutralMode(NeutralModeValue.Brake);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public KrakenTestFSMState getCurrentState() {
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
		currentState = KrakenTestFSMState.IDLE_STOP;
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

		switch (currentState) {
			case IDLE_STOP:
				handleIdleState(input);
				break;
			case RUN_MOTOR:
				handleRunState(input);
				break;
			case RUN_FULL:
				handleFullSpeed(input);
				break;
			case RUN_HALF:
				handleHalfSpeed(input);
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
	private KrakenTestFSMState nextState(TeleopInput input) {
		if (input == null) {
			return KrakenTestFSMState.IDLE_STOP;
		}
		switch (currentState) {
			case IDLE_STOP:
				if (input.isRunMotorPressed()) {
					return KrakenTestFSMState.RUN_MOTOR;
				}
				if (input.isFullButtonPressed()) {
					return KrakenTestFSMState.RUN_FULL;
				}
				if (input.isHalfButtonPressed()) {
					return KrakenTestFSMState.RUN_HALF;
				}
				return KrakenTestFSMState.IDLE_STOP;

			case RUN_MOTOR:
				if (input.isRunMotorPressed()) {
					return KrakenTestFSMState.RUN_MOTOR;
				}
				if (input.isFullButtonPressed()) {
					return KrakenTestFSMState.RUN_FULL;
				}
				if (input.isHalfButtonPressed()) {
					return KrakenTestFSMState.RUN_HALF;
				}
				return KrakenTestFSMState.IDLE_STOP;

			case RUN_FULL:
				if (input.isRunMotorPressed()) {
					return KrakenTestFSMState.RUN_MOTOR;
				}
				if (input.isFullButtonPressed()) {
					return KrakenTestFSMState.RUN_FULL;
				}
				if (input.isHalfButtonPressed()) {
					return KrakenTestFSMState.RUN_HALF;
				}
				return KrakenTestFSMState.IDLE_STOP;

			case RUN_HALF:
				if (input.isRunMotorPressed()) {
					return KrakenTestFSMState.RUN_MOTOR;
				}
				if (input.isFullButtonPressed()) {
					return KrakenTestFSMState.RUN_FULL;
				}
				if (input.isHalfButtonPressed()) {
					return KrakenTestFSMState.RUN_HALF;
				}
				return KrakenTestFSMState.IDLE_STOP;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		testMotor.set(0);
	}

	/**
	 * Handle behavior in INTAKING state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRunState(TeleopInput input) {
		testMotor.set(RUN_SPEED);
	}

	/**
	 * Handle behavior in INTAKING state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFullSpeed(TeleopInput input) {
		testMotor.set(FULL_SPEED);
	}

	/**
	 * Handle behavior in INTAKING state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleHalfSpeed(TeleopInput input) {
		testMotor.set(HALF_SPEED);
	}
}
