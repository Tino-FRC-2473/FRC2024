package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

public class IntakeFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum IntakeFSMState {
		IDLE_STOP,
		INTAKING,
		OUTTAKING
	}

	private static final float INTAKE_POWER = 0.1f;
	private static final float OUTTAKE_POWER = -INTAKE_POWER;
	private static final float AUTO_INTAKING_TIME = 2.0f;
	private static final float AUTO_OUTTAKING_TIME = 2.0f;

	/* ======================== Private variables ======================== */
	private IntakeFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax intakeMotor;
	private DigitalInput breakBeamSwitch;

	private boolean autoIntakingTimerStarted;
	private double autoIntakingTimerStart;
	private boolean autoOuttakingTimerStarted;
	private double autoOuttakingTimerStart;

	private Timer intakeTimer;


	/* ======================== Constructor ======================== */
	/**
	 * Create IntakeFSM and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public IntakeFSM() {
		// Perform hardware init
		intakeTimer = new Timer();

		intakeMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_INTAKE_MOTOR,
										CANSparkMax.MotorType.kBrushless);

		intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		breakBeamSwitch = new DigitalInput(HardwareMap.INPUT_BREAK_BEAM_PORT);


		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public IntakeFSMState getCurrentState() {
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
		currentState = IntakeFSMState.IDLE_STOP;

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

		SmartDashboard.putBoolean("Brake Beam Switch DIO PORT 0", hasNote());

		switch (currentState) {
			case IDLE_STOP:
				handleIdleState(input);
				break;

			case INTAKING:
				handleIntakingState(input);
				break;

			case OUTTAKING:
				handleOuttakingState(input);
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
	private IntakeFSMState nextState(TeleopInput input) {
		if (input == null) {
			return IntakeFSMState.IDLE_STOP;
		}
		switch (currentState) {
			case IDLE_STOP:
				if (input.isOuttakeButtonPressed() && !input.isIntakeButtonPressed() && hasNote()) {
					return IntakeFSMState.OUTTAKING;
				}

				if (input.isIntakeButtonPressed() && !input.isOuttakeButtonPressed()
					&& !hasNote()) {
					return IntakeFSMState.INTAKING;
				}

				if (input.isIntakeButtonPressed() == input.isOuttakeButtonPressed()) {
					return IntakeFSMState.IDLE_STOP;
				}

			case INTAKING:
				if (input.isIntakeButtonPressed() && !input.isOuttakeButtonPressed()
					&& !hasNote()) {
					return IntakeFSMState.INTAKING;
				}

				if (!input.isIntakeButtonPressed() || input.isOuttakeButtonPressed()
					|| hasNote()) {
					return IntakeFSMState.IDLE_STOP;
				}
			case OUTTAKING:
				if (!input.isIntakeButtonPressed() && input.isOuttakeButtonPressed()) {
					return IntakeFSMState.OUTTAKING;
				}

				if (!input.isOuttakeButtonPressed() || input.isIntakeButtonPressed()) {
					return IntakeFSMState.IDLE_STOP;
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
		intakeMotor.set(0);
	}
	/**
	 * Handle behavior in INTAKING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakingState(TeleopInput input) {
		intakeMotor.set(INTAKE_POWER);
	}

	/**
	 * Handle behavior in OUTTAKING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakingState(TeleopInput input) {
		intakeMotor.set(OUTTAKE_POWER);
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
	 * Performs action for auto Outtaking.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoOuttakingState() {

		if (!autoOuttakingTimerStarted) {
			autoOuttakingTimerStarted = true;
			autoOuttakingTimerStart = intakeTimer.get();
		}

		if ((autoOuttakingTimerStarted
			&& !intakeTimer.hasElapsed(autoOuttakingTimerStart + AUTO_OUTTAKING_TIME))
			|| !breakBeamSwitch.get()) {

			intakeMotor.set(OUTTAKE_POWER);
		} else {
			intakeMotor.set(0);
			autoOuttakingTimerStarted = false;

			return true;
		}

		return false;
	}

	/**
	 * Performs action for auto Intaking.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoIntakingState() {

		if (!autoIntakingTimerStarted) {
			autoIntakingTimerStarted = true;
			autoIntakingTimerStart = intakeTimer.get();
		}

		if ((autoIntakingTimerStarted
			&& !intakeTimer.hasElapsed(autoIntakingTimerStart + AUTO_INTAKING_TIME))
			|| !breakBeamSwitch.get()) {
			intakeMotor.set(INTAKE_POWER);
		} else {
			intakeMotor.set(0);
			autoIntakingTimerStarted = false;

			return true;
		}

		return false;
	}

	private boolean hasNote() {
		return !breakBeamSwitch.get();
	}
}
