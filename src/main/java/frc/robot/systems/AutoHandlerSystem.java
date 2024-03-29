package frc.robot.systems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoHandlerSystem {
	/* ======================== Constants ======================== */
	// Auto FSM state definitions
	public enum AutoFSMState {
		LEAVE,
		DRIVE_TO_SCORE,
		PICK_UP_1,
		PICK_UP_2,
		PICK_UP_3,
		PICK_UP_4,
		SHOOTER_STATE,
		PENDING
	}

	public enum AutoPath {
		PATH1, // score and leave
		PATH2, // score multiple times
		PATH3 // tbd emergency path
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

	private static final AutoFSMState[] PATH1 = new AutoFSMState[]{
		AutoFSMState.DRIVE_TO_SCORE, AutoFSMState.SHOOTER_STATE, AutoFSMState.LEAVE};

	private static final AutoFSMState[] PATH2 = new AutoFSMState[]{
		AutoFSMState.DRIVE_TO_SCORE, AutoFSMState.SHOOTER_STATE, AutoFSMState.PICK_UP_1,
		AutoFSMState.DRIVE_TO_SCORE, AutoFSMState.SHOOTER_STATE, AutoFSMState.PICK_UP_2,
		AutoFSMState.DRIVE_TO_SCORE, AutoFSMState.SHOOTER_STATE, AutoFSMState.PICK_UP_3,
		AutoFSMState.DRIVE_TO_SCORE, AutoFSMState.SHOOTER_STATE, AutoFSMState.PICK_UP_4};

	private static final AutoFSMState[] PATH3 = new AutoFSMState[]{AutoFSMState.SHOOTER_STATE};

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

		if (path == AutoPath.PATH1) {
			currentStateList = PATH1;
		} else if (path == AutoPath.PATH2) {
			currentStateList = PATH2;
		} else if (path == AutoPath.PATH3) {
			currentStateList = PATH3;
		}
		currentStateIndex = 0;
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
			case LEAVE:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.LEAVE);
				break;
			case PICK_UP_1:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.PICK_UP_1);
				break;
			case DRIVE_TO_SCORE:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.DRIVE_TO_SCORE);
				break;
			case PICK_UP_2:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.PICK_UP_2);
				break;
			case PICK_UP_3:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.PICK_UP_3);
				break;
			case PICK_UP_4:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.PICK_UP_4);
				break;
			case SHOOTER_STATE:
				isCurrentStateFinished = shooterFSM.updateAutonomous(AutoFSMState.SHOOTER_STATE);
				break;
			case PENDING:
				isCurrentStateFinished = driveSystem.updateAutonomous(AutoFSMState.PENDING);
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
