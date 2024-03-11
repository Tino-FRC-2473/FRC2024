package frc.robot.systems;

public class AutoHandlerSystem {
	/* ======================== Constants ======================== */
	// Auto FSM state definitions
	public enum AutoFSMState {
		SHOOT,
		DRIVE_TO_NOTE,
		DRIVE_TO_SPEAKER
	}
	public enum AutoPath {
		PATH1,
		PATH2,
		PATH3
	}

	/* ======================== Private variables ======================== */
	//Contains the sequential list of states in the current auto path that must be executed
	private AutoFSMState[] currentStateList;

	//The index in the currentStateList where the currentState is at
	private int currentStateIndex;

	//FSM Systems that the autoHandlerFSM uses
	private IntakeFSM intakeFSM;
	private MBRShooterFSM shooterFSM;
	private PivotFSM pivotFSM;

	//Predefined auto paths
	private static final AutoFSMState[] PATH1 = new AutoFSMState[]{
		AutoFSMState.DRIVE_TO_NOTE, AutoFSMState.DRIVE_TO_SPEAKER, AutoFSMState.SHOOT};

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state.
	 * Initializes any subsystems such as driveFSM, armFSM, ect.
	 * @param fsm1 the first subsystem that the auto handler will call functions on
	 * @param fsm2 the second subsystem that the auto handler will call functions on
	 * @param fsm3 the third subsystem that the auto handler will call functions on
	 */
	public AutoHandlerSystem(IntakeFSM fsm1, MBRShooterFSM fsm2, PivotFSM fsm3) {
		intakeFSM = fsm1;
		shooterFSM = fsm2;
		pivotFSM = fsm3;
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public AutoFSMState getCurrentState() {
		return currentStateList[currentStateIndex];
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 * @param path the auto path to be executed
	 */
	public void reset(AutoPath path) {
		intakeFSM.reset();
		shooterFSM.reset();
		pivotFSM.reset();
		currentStateList = PATH1; //default
		currentStateIndex = 0;
		if (path == AutoPath.PATH1) {
			currentStateList = PATH1;
		}
	}

	/**
	 * This function runs the auto's current state.
	 */
	public void update() {
		if (currentStateIndex >= currentStateList.length) {
			return;
		}

		boolean isCurrentStateFinished;
		System.out.println("In State: " + getCurrentState());
		switch (getCurrentState()) {
			case SHOOT:
				isCurrentStateFinished = intakeFSM.updateAutonomous(AutoFSMState.SHOOT)
					& shooterFSM.updateAutonomous(AutoFSMState.SHOOT)
					& pivotFSM.updateAutonomous(AutoFSMState.SHOOT);
				break;
			case DRIVE_TO_NOTE:
				//add driveFSM.updateAutonomous() & to the front of the bottom expression
				isCurrentStateFinished = (pivotFSM.updateAutonomous(AutoFSMState.DRIVE_TO_NOTE)
					&& intakeFSM.updateAutonomous(AutoFSMState.DRIVE_TO_NOTE))
					& shooterFSM.updateAutonomous(AutoFSMState.DRIVE_TO_NOTE);
				break;
			case DRIVE_TO_SPEAKER:
				//add driveFSM.updateAutonomous() & to the front of the bottom expression
				isCurrentStateFinished = (pivotFSM.updateAutonomous(AutoFSMState.DRIVE_TO_SPEAKER)
					&& shooterFSM.updateAutonomous(AutoFSMState.DRIVE_TO_SPEAKER))
					& intakeFSM.updateAutonomous(AutoFSMState.DRIVE_TO_SPEAKER);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
		if (isCurrentStateFinished) {
			currentStateIndex++;
		}
	}
}
