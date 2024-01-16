package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.AutoPathChooser;

public class AutoHandlerSystem {
	/* ======================== Constants ======================== */
	// Auto FSM state definitions
	public enum AutoFSMState {
		DRIVE_PATH_1,
		DRIVE_PATH_2,
		DRIVE_PATH_3,
		DRIVE_PATH_4_STATE_1,
		DRIVE_PATH_4_STATE_2,
		DRIVE_PATH_5
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
	private static final AutoFSMState[] PATH1 = new AutoFSMState[]{
		AutoFSMState.DRIVE_PATH_1};

	private static final AutoFSMState[] PATH2 = new AutoFSMState[]{
		AutoFSMState.DRIVE_PATH_2};

	private static final AutoFSMState[] PATH3 = new AutoFSMState[]{
		AutoFSMState.DRIVE_PATH_3};

	private static final AutoFSMState[] PATH4 = new AutoFSMState[]{
		AutoFSMState.DRIVE_PATH_4_STATE_1, AutoFSMState.DRIVE_PATH_4_STATE_2};

	private static final AutoFSMState[] PATH5 = new AutoFSMState[]{
		AutoFSMState.DRIVE_PATH_5};

	private static final AutoFSMState[] PATH6 = new AutoFSMState[]{};
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
		if (path == AutoPath.PATH1) {
			currentStateList = PATH1;
		} else if (path == AutoPath.PATH2) {
			currentStateList = PATH2;
		} else if (path == AutoPath.PATH3) {
			currentStateList = PATH3;
		} else if (path == AutoPath.PATH4) {
			currentStateList = PATH4;
		} else if (path == AutoPath.PATH5) {
			currentStateList = PATH5;
		} else if (path == AutoPath.PATH6) {
			currentStateList = PATH6;
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
		SmartDashboard.putString("In Auto State: ", "" + getCurrentState());
		switch (getCurrentState()) {
			case DRIVE_PATH_1:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.DRIVE_PATH_1);
				break;
			case DRIVE_PATH_2:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.DRIVE_PATH_2);
				break;
			case DRIVE_PATH_3:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.DRIVE_PATH_3);
				break;
			case DRIVE_PATH_4_STATE_1:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.DRIVE_PATH_4_STATE_1);
				break;
			case DRIVE_PATH_4_STATE_2:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.DRIVE_PATH_4_STATE_2);
				break;
			case DRIVE_PATH_5:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.DRIVE_PATH_5);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
		if (isCurrentStateFinished) {
			currentStateIndex++;
		}
	}
}
