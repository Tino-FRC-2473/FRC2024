package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.AutoPathChooser;

public class AutoHandlerSystem {
	/* ======================== Constants ======================== */
	// Auto FSM state definitions
	public enum AutoFSMState {
		SVR_DRIVE_PATH_1,
		SVR_DRIVE_PATH_2,
		SVR_DRIVE_PATH_3,
		SVR_DRIVE_PATH_4_STATE_1,
		SVR_DRIVE_PATH_4_STATE_2,
		SVR_DRIVE_PATH_5,
		MBR_DRIVE_PATH_1_STATE_1,
		MBR_DRIVE_PATH_1_STATE_2,
		MBR_DRIVE_PATH_2_STATE_1,
		MBR_DRIVE_PATH_2_STATE_2,
		MBR_DRIVE_PATH_3_STATE_1,
		MBR_DRIVE_PATH_3_STATE_2,
		MBR_DRIVE_PATH_4_STATE_1,
		MBR_DRIVE_PATH_4_STATE_2,
		MBR_DRIVE_PATH_4_STATE_3,
		MBR_DRIVE_PATH_5,
		MBR_DRIVE_PATH_6,
		MBR_DRIVE_PATH_7,
		MBR_DRIVE_PATH_8_STATE_1,
		MBR_DRIVE_PATH_8_STATE_2,
		MBR_DRIVE_PATH_9_STATE_1,
		MBR_DRIVE_PATH_9_STATE_2,
		SHOOTER_STATE_1,
		SHOOTER_STATE_2,
		SHOOTER_STATE_3,
		PENDING
	}
	public enum AutoPath {
		SVR_PATH1,
		SVR_PATH2,
		SVR_PATH3,
		SVR_PATH4,
		SVR_PATH5,
		MBR_PATH1,
		MBR_PATH2,
		MBR_PATH3,
		MBR_PATH4,
		MBR_PATH5,
		MBR_PATH6,
		MBR_PATH7,
		MBR_PATH8,
		MBR_PATH9
	}

	/* ======================== Private variables ======================== */
	//Contains the sequential list of states in the current auto path that must be executed
	private AutoFSMState[] currentStateList;

	//The index in the currentStateList where the currentState is at
	private int currentStateIndex;

	//FSM Systems that the autoHandlerFSM uses
	private DriveFSMSystem driveSystem;
	private KitBotShooterFSM shooterFSM;

	//Predefined auto paths
	private static final AutoFSMState[] SVR_PATH1 = new AutoFSMState[]{
		AutoFSMState.SVR_DRIVE_PATH_1};

	private static final AutoFSMState[] SVR_PATH2 = new AutoFSMState[]{
		AutoFSMState.SVR_DRIVE_PATH_2};

	private static final AutoFSMState[] SVR_PATH3 = new AutoFSMState[]{
		AutoFSMState.SVR_DRIVE_PATH_3};

	private static final AutoFSMState[] SVR_PATH4 = new AutoFSMState[]{
		AutoFSMState.PENDING, AutoFSMState.SVR_DRIVE_PATH_4_STATE_1,
		AutoFSMState.SVR_DRIVE_PATH_4_STATE_2};

	private static final AutoFSMState[] SVR_PATH5 = new AutoFSMState[]{
		AutoFSMState.SVR_DRIVE_PATH_5};

	private static final AutoFSMState[] MBR_PATH1 = new AutoFSMState[]{
		AutoFSMState.MBR_DRIVE_PATH_1_STATE_1, AutoFSMState.MBR_DRIVE_PATH_1_STATE_2};

	private static final AutoFSMState[] MBR_PATH2 = new AutoFSMState[]{
		AutoFSMState.MBR_DRIVE_PATH_2_STATE_1, AutoFSMState.MBR_DRIVE_PATH_2_STATE_2};

	private static final AutoFSMState[] MBR_PATH3 = new AutoFSMState[]{
		AutoFSMState.MBR_DRIVE_PATH_3_STATE_1, AutoFSMState.MBR_DRIVE_PATH_3_STATE_2};

	private static final AutoFSMState[] MBR_PATH4 = new AutoFSMState[]{
		AutoFSMState.MBR_DRIVE_PATH_4_STATE_1, AutoFSMState.MBR_DRIVE_PATH_4_STATE_2,
		AutoFSMState.MBR_DRIVE_PATH_4_STATE_3};

	private static final AutoFSMState[] MBR_PATH5 = new AutoFSMState[]{
		AutoFSMState.MBR_DRIVE_PATH_5};

	private static final AutoFSMState[] MBR_PATH6 = new AutoFSMState[]{
		AutoFSMState.MBR_DRIVE_PATH_6};

	private static final AutoFSMState[] MBR_PATH7 = new AutoFSMState[]{
		AutoFSMState.MBR_DRIVE_PATH_7};

	private static final AutoFSMState[] MBR_PATH8 = new AutoFSMState[]{
		AutoFSMState.MBR_DRIVE_PATH_8_STATE_1, AutoFSMState.MBR_DRIVE_PATH_8_STATE_2};

	private static final AutoFSMState[] MBR_PATH9 = new AutoFSMState[]{
		AutoFSMState.MBR_DRIVE_PATH_9_STATE_1, AutoFSMState.MBR_DRIVE_PATH_9_STATE_2};
	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state.
	 * Initializes any subsystems such as driveFSM, armFSM, ect.
	 * @param fsm1 the first subsystem that the auto handler will call functions on
	 * @param fsm2 the second subsystem that the auto handler will call functions on
	 */
	public AutoHandlerSystem(DriveFSMSystem fsm1, KitBotShooterFSM fsm2) {
		driveSystem = fsm1;
		shooterFSM = fsm2;
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
		shooterFSM.reset();

		currentStateIndex = 0;
		if (path == AutoPath.SVR_PATH1) {
			currentStateList = SVR_PATH1;
		} else if (path == AutoPath.SVR_PATH2) {
			currentStateList = SVR_PATH2;
		} else if (path == AutoPath.SVR_PATH3) {
			currentStateList = SVR_PATH3;
		} else if (path == AutoPath.SVR_PATH4) {
			currentStateList = SVR_PATH4;
		} else if (path == AutoPath.SVR_PATH5) {
			currentStateList = SVR_PATH5;
		} else if (path == AutoPath.MBR_PATH1) {
			currentStateList = MBR_PATH1;
		} else if (path == AutoPath.MBR_PATH2) {
			currentStateList = MBR_PATH2;
		} else if (path == AutoPath.MBR_PATH3) {
			currentStateList = MBR_PATH3;
		} else if (path == AutoPath.MBR_PATH4) {
			currentStateList = MBR_PATH4;
		} else if (path == AutoPath.MBR_PATH5) {
			currentStateList = MBR_PATH5;
		} else if (path == AutoPath.MBR_PATH6) {
			currentStateList = MBR_PATH6;
		} else if (path == AutoPath.MBR_PATH7) {
			currentStateList = MBR_PATH7;
		} else if (path == AutoPath.MBR_PATH8) {
			currentStateList = MBR_PATH8;
		} else if (path == AutoPath.MBR_PATH9) {
			currentStateList = MBR_PATH9;
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
			case SVR_DRIVE_PATH_1:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.SVR_DRIVE_PATH_1);
				break;
			case SVR_DRIVE_PATH_2:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.SVR_DRIVE_PATH_2);
				break;
			case SVR_DRIVE_PATH_3:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.SVR_DRIVE_PATH_3);
				break;
			case SVR_DRIVE_PATH_4_STATE_1:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.SVR_DRIVE_PATH_4_STATE_1);
				break;
			case SVR_DRIVE_PATH_4_STATE_2:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.SVR_DRIVE_PATH_4_STATE_2);
				break;
			case SVR_DRIVE_PATH_5:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.SVR_DRIVE_PATH_5);
				break;
			case MBR_DRIVE_PATH_1_STATE_1:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_1_STATE_1);
			case MBR_DRIVE_PATH_1_STATE_2:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_1_STATE_2);
			case MBR_DRIVE_PATH_2_STATE_1:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_2_STATE_1);
			case MBR_DRIVE_PATH_2_STATE_2:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_2_STATE_2);
			case MBR_DRIVE_PATH_3_STATE_1:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_3_STATE_1);
			case MBR_DRIVE_PATH_3_STATE_2:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_3_STATE_2);
			case MBR_DRIVE_PATH_4_STATE_1:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_4_STATE_1);
			case MBR_DRIVE_PATH_4_STATE_2:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_4_STATE_2);
			case MBR_DRIVE_PATH_5:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_5);
			case MBR_DRIVE_PATH_6:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_6);
			case MBR_DRIVE_PATH_7:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_7);
			case MBR_DRIVE_PATH_8_STATE_1:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_8_STATE_1);
			case MBR_DRIVE_PATH_8_STATE_2:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_8_STATE_2);
			case MBR_DRIVE_PATH_9_STATE_1:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_9_STATE_1);
			case MBR_DRIVE_PATH_9_STATE_2:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.MBR_DRIVE_PATH_9_STATE_2);
			case SHOOTER_STATE_1:
				isCurrentStateFinished = shooterFSM.updateAutonomous(
					AutoFSMState.SHOOTER_STATE_1);
				break;
			case SHOOTER_STATE_2:
				isCurrentStateFinished = shooterFSM.updateAutonomous(
					AutoFSMState.SHOOTER_STATE_2);
				break;
			case SHOOTER_STATE_3:
				isCurrentStateFinished = shooterFSM.updateAutonomous(
					AutoFSMState.SHOOTER_STATE_3);
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
