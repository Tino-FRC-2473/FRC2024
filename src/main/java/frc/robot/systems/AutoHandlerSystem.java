package frc.robot.systems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveConstants.AutoConstants;

import java.util.ArrayList;

public class AutoHandlerSystem {
	/* ======================== Constants ======================== */
	// Auto FSM state definitions
	public enum AutoFSMState {
		/* --------------------------- SVR --------------------------- */
		// LEAVE,
		// DRIVE_TO_SCORE,
		// PICK_UP_1,
		// PICK_UP_2,
		// PICK_UP_3,
		// PICK_UP_4,
		// SHOOTER_STATE,
		// PENDING,
		// SHOOTER_STATE_FAST,
		// RUN_OVER_NOTES
		/* --------------------------- SVR --------------------------- */
		DEFAULT,
		SPEAKER,
		NOTE1,
		NOTE2,
		NOTE3,
		NOTE4,
		NOTE5,
		NOTE6,
		NOTE7,
		NOTE8,
		SHOOT,
		SHOOT_PRELOADED
	}

	/* --------------------------- SVR --------------------------- */
	// public enum AutoPath {
	// 	PATH1, // score and leave
	// 	PATH2, // score multiple times
	// 	PATH3, // tbd emergency path
	// 	PATH4,
	// 	PATH5
	// }
	/* --------------------------- SVR --------------------------- */

	/* ======================== Private variables ======================== */
	//Contains the sequential list of states in the current auto path that must be executed
	private ArrayList<AutoFSMState> currentStateList = new ArrayList<>();

	//The index in the currentStateList where the currentState is at
	private int currentStateIndex;

	//FSM Systems that the autoHandlerFSM uses
	private DriveFSMSystem driveSystem;
	private MBRFSMv2 mechSystem;

	//Predefined auto paths

	/* --------------------------- SVR --------------------------- */
	// private static final AutoFSMState[] PATH1 = new AutoFSMState[]{
	// 	AutoFSMState.DRIVE_TO_SCORE, AutoFSMState.SHOOTER_STATE, AutoFSMState.LEAVE};

	// private static final AutoFSMState[] PATH2 = new AutoFSMState[]{
	// 	AutoFSMState.DRIVE_TO_SCORE, AutoFSMState.SHOOTER_STATE, AutoFSMState.PICK_UP_1,
	// 	AutoFSMState.DRIVE_TO_SCORE, AutoFSMState.SHOOTER_STATE, AutoFSMState.PICK_UP_2,
	// 	AutoFSMState.DRIVE_TO_SCORE, AutoFSMState.SHOOTER_STATE, AutoFSMState.PICK_UP_3,
	// 	AutoFSMState.DRIVE_TO_SCORE, AutoFSMState.SHOOTER_STATE, AutoFSMState.PICK_UP_4};

	// private static final AutoFSMState[] PATH3 = new AutoFSMState[]{AutoFSMState.DRIVE_TO_SCORE,
	// 	AutoFSMState.SHOOTER_STATE};

	// private static final AutoFSMState[] PATH4 = new AutoFSMState[]{AutoFSMState.DRIVE_TO_SCORE,
	// 	AutoFSMState.SHOOTER_STATE_FAST, AutoFSMState.RUN_OVER_NOTES};

	// private static final AutoFSMState[] PATH5 = new AutoFSMState[]{AutoFSMState.RUN_OVER_NOTES};
	/* --------------------------- SVR --------------------------- */

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state.
	 * Initializes any subsystems such as driveFSM, armFSM, ect.
	 * @param fsm1 the first subsystem that the auto handler will call functions on
	 * @param fsm2 the second subsystem that the auto handler will call functions on
	 */
	public AutoHandlerSystem(DriveFSMSystem fsm1, MBRFSMv2 fsm2) {
		driveSystem = fsm1;
		mechSystem = fsm2;
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public AutoFSMState getCurrentState() {
		return currentStateList.get(currentStateIndex);
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
	public void reset(String path) {
		driveSystem.resetAutonomus();
		mechSystem.reset();

		/* --------------------------- SVR --------------------------- */
		// if (path == AutoPath.PATH1) {
		// 	currentStateList = PATH1;
		// } else if (path == AutoPath.PATH2) {
		// 	currentStateList = PATH2;
		// } else if (path == AutoPath.PATH3) {
		// 	currentStateList = PATH3;
		// } else if (path == AutoPath.PATH4) {
		// 	currentStateList = PATH4;
		// } else if (path == AutoPath.PATH5) {
		// 	currentStateList = PATH5;
		// }
		// currentStateIndex = 0;
		/* --------------------------- SVR --------------------------- */

		currentStateList.clear();
		currentStateList.add(AutoFSMState.SHOOT_PRELOADED);
		currentStateList.add(AutoFSMState.DEFAULT);
		if (path.contains("PROT") || path.contains("MISC") || path.contains("MIDF")) {
			if (!path.contains("PROT")) {
				currentStateList.add(AutoFSMState.SPEAKER);
			}
			String notes = path.substring((int) AutoConstants.N_8 + 1 + 1); // 10
			for (int i = 0; i < notes.length(); i++) {
				int id = notes.charAt(i) - '0';
				if (id == 1) {
					currentStateList.add(AutoFSMState.NOTE1);
				} else if (id == 2) {
					currentStateList.add(AutoFSMState.NOTE2);
				} else if (id == AutoConstants.N_3) {
					currentStateList.add(AutoFSMState.NOTE3);
				} else if (id == AutoConstants.N_4) {
					currentStateList.add(AutoFSMState.NOTE4);
				} else if (id == AutoConstants.N_5) {
					currentStateList.add(AutoFSMState.NOTE5);
				} else if (id == AutoConstants.N_6) {
					currentStateList.add(AutoFSMState.NOTE6);
				} else if (id == AutoConstants.N_7) {
					currentStateList.add(AutoFSMState.NOTE7);
				} else if (id == AutoConstants.N_8) {
					currentStateList.add(AutoFSMState.NOTE8);
				}
				currentStateList.add(AutoFSMState.SPEAKER);
				currentStateList.add(AutoFSMState.SHOOT);
			}
		}
		//add <LEAVE ZONE> Path to currentStateList
		currentStateIndex = 0;
	}

	/**
	 * This function runs the auto's current state.
	 */
	public void update() {
		if (currentStateIndex >= currentStateList.size()) {
			return;
		}
		boolean isCurrentStateFinished;
		SmartDashboard.putString("In Auto State: ", "" + getCurrentState());
		switch (getCurrentState()) {
			case DEFAULT:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.DEFAULT);
				break;
			case SPEAKER:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.SPEAKER)
					& mechSystem.updateAutonomous(AutoFSMState.SPEAKER);
				break;
			case NOTE1:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.NOTE1)
					& mechSystem.updateAutonomous(AutoFSMState.NOTE1);
				break;
			case NOTE2:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.NOTE2)
					& mechSystem.updateAutonomous(AutoFSMState.NOTE2);
				break;
			case NOTE3:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.NOTE3)
					& mechSystem.updateAutonomous(AutoFSMState.NOTE3);
				break;
			case NOTE4:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.NOTE4)
					& mechSystem.updateAutonomous(AutoFSMState.NOTE4);
				break;
			case NOTE5:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.NOTE5)
					& mechSystem.updateAutonomous(AutoFSMState.NOTE5);
				break;
			case NOTE6:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.NOTE6)
					& mechSystem.updateAutonomous(AutoFSMState.NOTE6);
				break;
			case NOTE7:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.NOTE7)
					& mechSystem.updateAutonomous(AutoFSMState.NOTE7);
				break;
			case NOTE8:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.NOTE8)
					& mechSystem.updateAutonomous(AutoFSMState.NOTE8);
				break;
			case SHOOT:
				isCurrentStateFinished = mechSystem.updateAutonomous(AutoFSMState.SHOOT);
				break;
			case SHOOT_PRELOADED:
				isCurrentStateFinished = mechSystem.updateAutonomous(AutoFSMState.SHOOT_PRELOADED);
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
