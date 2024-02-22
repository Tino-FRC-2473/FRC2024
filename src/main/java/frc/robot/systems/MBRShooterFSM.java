package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

public class MBRShooterFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		IDLE_STOP,
		SHOOTING
	}

	private static final float SHOOTING_POWER = 0.7f;
	private static final float SHOOTING_TIME = 2.0f;
	private boolean buttonToggle = false;
	private boolean buttonPressedLastFrame = false;


	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;

	private boolean autoShootingTimerStarted;
	private double autoShootingTimerStart;

	private Timer shootingTimer;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public MBRShooterFSM() {
		// Perform hardware init
		shootingTimer = new Timer();

		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_LSHOOTER_MOTOR,
										CANSparkMax.MotorType.kBrushless);

		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_RSHOOTER_MOTOR,
										CANSparkMax.MotorType.kBrushless);
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
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.IDLE_STOP;

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

		if (input.isShootButtonPressed() && !buttonPressedLastFrame) {
			buttonToggle = !buttonToggle;
			buttonPressedLastFrame = true;
		} else if (!input.isShootButtonPressed()) {
			buttonPressedLastFrame = false;
		}

		switch (currentState) {
			case IDLE_STOP:
				handleIdleState(input);
				break;

			case SHOOTING:
				handleShootingState(input);
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
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case IDLE_STOP:
				if (buttonToggle) {
					return FSMState.SHOOTING;
				} else {
					return FSMState.IDLE_STOP;
				}
			case SHOOTING:
				if (buttonToggle) {
					return FSMState.SHOOTING;
				} else {
					return FSMState.IDLE_STOP;
				}

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
		leftMotor.set(0);
		rightMotor.set(0);
	}
	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleShootingState(TeleopInput input) {
		leftMotor.set(-SHOOTING_POWER); //dont forget the "-" sign
		rightMotor.set(SHOOTING_POWER);
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
	 * Performs action for auto Shooting.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoShootingState() {
		if (!autoShootingTimerStarted) {
			autoShootingTimerStarted = true;
			autoShootingTimerStart = shootingTimer.get();
		}

		if (autoShootingTimerStarted
			&& !shootingTimer.hasElapsed(autoShootingTimerStart + SHOOTING_TIME)) {
			leftMotor.set(-SHOOTING_POWER);
			rightMotor.set(SHOOTING_POWER);
		} else {
			leftMotor.set(0);
			rightMotor.set(0);

			autoShootingTimerStarted = false;

			return true;
		}

		return false;
	}
}
