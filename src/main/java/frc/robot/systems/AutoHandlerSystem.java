package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.AutoPathChooser;

public class AutoHandlerSystem {
	/* ======================== Constants ======================== */
	// Auto FSM state definitions
	public enum AutoFSMState {
		TURN_TO_SPEAKER,
		LEAVE_SPEAKER,
		LEAVE,
		PENDING
	}
	public enum AutoPath {
		PATH1,
		PATH2,
		PATH3,
		PATH4,
		PATH5,
		PATH6
	}

	/* ======================== Private variables ======================== */
	//Contains the sequential list of states in the current auto path that must be executed
	private AutoFSMState[] currentStateList;

	//The index in the currentStateList where the currentState is at
	private int currentStateIndex;

	//FSM Systems that the autoHandlerFSM uses
	private DriveFSMSystem driveSystem;

	//Predefined auto paths

	private static final AutoFSMState[] PATH3 = new AutoFSMState[]{
		AutoFSMState.TURN_TO_SPEAKER, AutoFSMState.LEAVE_SPEAKER};
	private static final AutoFSMState[] PATH5 = new AutoFSMState[]{
		AutoFSMState.LEAVE};


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state.
	 * Initializes any subsystems such as driveFSM, armFSM, ect.
	 * @param fsm1 the first subsystem that the auto handler will call functions on
	 */
	public AutoHandlerSystem(DriveFSMSystem fsm1) {
		driveSystem = fsm1;
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

		driveSystem.resetAutonomus();

		currentStateIndex = 0;
		if (path == AutoPath.PATH3) {
			currentStateList = PATH3;
		} else if (path == AutoPath.PATH5) {
			currentStateList = PATH5;
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
		SmartDashboard.putString("In Auto State: ", "" + getCurrentState());
		switch (getCurrentState()) {
			case TURN_TO_SPEAKER:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.TURN_TO_SPEAKER);
				break;
			case LEAVE_SPEAKER:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.LEAVE_SPEAKER);
				break;
			case LEAVE:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.LEAVE);
				break;
			case PENDING:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.PENDING);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
		if (isCurrentStateFinished) {
			currentStateIndex++;
			driveSystem.setCurrentPointInPath(0);
		}
	}
}
