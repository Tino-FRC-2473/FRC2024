// package frc.robot.systems;

// import com.ctre.phoenix6.signals.NeutralModeValue;

// // WPILib Imports

// // Third party Hardware Imports
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// // Robot Imports
// import frc.robot.TeleopInput;
// import frc.robot.HardwareMap;
// import frc.robot.systems.AutoHandlerSystem.AutoFSMState;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// public class IntakeFSM {
// 	/* ======================== Constants ======================== */
// 	// FSM state definitions
// 	public enum IntakeFSMState {
// 		IDLE_STOP,
// 		INTAKING,
// 		OUTTAKING
// 	}


// 	private static final float INTAKE_POWER = 0.1f;
// 	private static final float OUTTAKE_POWER = -0.1f;
// 	private static final float HOLDING_POWER = 0.03f;
// 	private static final float AUTO_INTAKING_TIME = 2.0f;
// 	private static final float AUTO_OUTTAKING_TIME = 2.0f;
// 	private static final int AVERAGE_SIZE = 7;
// 	private static final float CURRENT_THRESHOLD = 15.0f;


// 	/* ======================== Private variables ======================== */
// 	private IntakeFSMState currentState;

// 	// Hardware devices should be owned by one and only one system. They must
// 	// be private to their owner system and may not be used elsewhere.
// 	private TalonFX intakeMotor;

// 	private boolean autoIntakingTimerStarted;
// 	private double autoIntakingTimerStart;
// 	private boolean autoOuttakingTimerStarted;
// 	private double autoOuttakingTimerStart;
// 	private double[] currLogs;
// 	private int tick = 0;
// 	private boolean holding = false;



// 	/* ======================== Constructor ======================== */
// 	/**
// 	 * Create IntakeFSM and initialize to starting state. Also perform any
// 	 * one-time initialization or configuration of hardware required. Note
// 	 * the constructor is called only once when the robot boots.
// 	 */
// 	public IntakeFSM() {
// 		// Perform hardware init

// 		intakeMotor = new TalonFX(HardwareMap.DEVICE_ID_INTAKE_MOTOR);
// 		intakeMotor.setNeutralMode(NeutralModeValue.Brake);


// 		currLogs = new double[AVERAGE_SIZE];
// 		// Reset state machine
// 		reset();
// 	}

// 	/* ======================== Public methods ======================== */
// 	/**
// 	 * Return current FSM state.
// 	 * @return Current FSM state
// 	 */
// 	public IntakeFSMState getCurrentState() {
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
// 		currentState = IntakeFSMState.IDLE_STOP;

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

// 		if (input == null) {
// 			return;
// 		}

// 		// SmartDashboard.putBoolean("Current value surpassed / hasNote", hasNote());
// 		SmartDashboard.putNumberArray("Current array values", currLogs);

// 		switch (currentState) {
// 			case IDLE_STOP:
// 				handleIdleState(input);
// 				break;

// 			case INTAKING:
// 				handleIntakingState(input);
// 				break;

// 			case OUTTAKING:
// 				handleOuttakingState(input);
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
// 		switch (autoState) {
// 			case DRIVE_TO_SPEAKER:
// 				return handleAutoIdleState();
// 			case DRIVE_TO_NOTE:
// 				return handleAutoIntakingState();
// 			case SHOOT:
// 				return handleAutoOuttakingState();
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
// 	private IntakeFSMState nextState(TeleopInput input) {
// 		if (input == null) {
// 			return IntakeFSMState.IDLE_STOP;
// 		}
// 		switch (currentState) {
// 			case IDLE_STOP:
// 				if (input.isOuttakeButtonPressed() && !input.isIntakeButtonPressed()) {
// 					return IntakeFSMState.OUTTAKING;
// 				}

// 				if (input.isIntakeButtonPressed() && !input.isOuttakeButtonPressed()
// 					&& !hasNote()) {
// 					return IntakeFSMState.INTAKING;
// 				}

// 				return IntakeFSMState.IDLE_STOP;

// 			case INTAKING:
// 				if (input.isIntakeButtonPressed() && !input.isOuttakeButtonPressed()
// 					&& !hasNote()) {
// 					return IntakeFSMState.INTAKING;
// 				}

// 				return IntakeFSMState.IDLE_STOP;
// 			case OUTTAKING:
// 				if (!input.isIntakeButtonPressed() && input.isOuttakeButtonPressed()) {
// 					return IntakeFSMState.OUTTAKING;
// 				}

// 				return IntakeFSMState.IDLE_STOP;

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
// 		// if (holding) {
// 		// 	intakeMotor.set(HOLDING_POWER);
// 		// } else {
// 		intakeMotor.set(0);
// 		// }
// 	}
// 	/**
// 	 * Handle behavior in INTAKING.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	private void handleIntakingState(TeleopInput input) {
// 		intakeMotor.set(INTAKE_POWER);
// 		currLogs[tick % AVERAGE_SIZE] = intakeMotor.getTorqueCurrent().getValueAsDouble();
// 		tick++;
// 		double avgcone = 0;
// 		for (int i = 0; i < AVERAGE_SIZE; i++) {
// 			avgcone += currLogs[i];
// 		}
// 		avgcone /= AVERAGE_SIZE;
// 		SmartDashboard.putNumber("avg current", avgcone);
// 		// if (avgcone > CURRENT_THRESHOLD) {
// 		// 	holding = true;
// 		// } else {
// 		// 	holding = false;
// 		// }
// 	}

// 	/**
// 	 * Handle behavior in OUTTAKING.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	private void handleOuttakingState(TeleopInput input) {
// 		for (int i = 0; i < AVERAGE_SIZE; i++) {
// 			currLogs[i] = 0;
// 		}
// 		// holding = false;
// 		intakeMotor.set(OUTTAKE_POWER);
// 	}


// 	/**
// 	 * Performs action for auto Outtaking.
// 	 * @return if the action carried out has finished executing
// 	 */
// 	private boolean handleAutoOuttakingState() {
// 		for (int i = 0; i < AVERAGE_SIZE; i++) {
// 			currLogs[i] = 0;
// 		}
// 		holding = false;
// 		intakeMotor.set(OUTTAKE_POWER);
// 		return true;
// 	}

// 	/**
// 	 * Performs action for auto Intaking.
// 	 * @return if the action carried out has finished executing
// 	 */
// 	private boolean handleAutoIntakingState() {
// 		intakeMotor.set(INTAKE_POWER);
// 		holding = true;
// 		return true;
// 	}

// 	private boolean handleAutoIdleState() {
// 		if (holding) {
// 			intakeMotor.set(HOLDING_POWER);
// 		} else {
// 			intakeMotor.set(0);
// 		}
// 		return true;
// 	}

// 	private boolean hasNote() {
// 		//return holding;
// 		return false;
// 	}
// }
