package frc.robot.systems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoHandlerSystem {
	/* ======================== Constants ======================== */
	// Auto FSM state definitions
	public enum AutoFSMState {
		TURN_TO_SPEAKER,
		LEAVE_SPEAKER,
		DRIVE_TO_SPEAKER,
		PICK_UP_1,
		PICK_UP_2,
		PICK_UP_3,
		LEAVE,
		SHOOTER_STATE,
		INTAKE_STATE,
		PENDING
	}

	public enum AutoPath {
		PATH1, // score and leave
		PATH2, // score multiple times
		PATH3, // just leave
		PATH4, // just score
		PATH5 // nothing
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
		AutoFSMState.TURN_TO_SPEAKER, AutoFSMState.LEAVE_SPEAKER};

	private static final AutoFSMState[] PATH2 = new AutoFSMState[]{
		AutoFSMState.TURN_TO_SPEAKER, AutoFSMState.PICK_UP_1,
		AutoFSMState.DRIVE_TO_SPEAKER, AutoFSMState.PICK_UP_2,
		AutoFSMState.DRIVE_TO_SPEAKER, AutoFSMState.PICK_UP_3,
		AutoFSMState.DRIVE_TO_SPEAKER};

	private static final AutoFSMState[] PATH3 = new AutoFSMState[]{
		AutoFSMState.LEAVE};

	private static final AutoFSMState[] PATH4 = new AutoFSMState[]{};

	private static final AutoFSMState[] PATH5 = new AutoFSMState[]{};

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
		//shooterFSM.reset();

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

			case TURN_TO_SPEAKER:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.TURN_TO_SPEAKER);
				break;
			case LEAVE_SPEAKER:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.LEAVE_SPEAKER);
				break;
			case PICK_UP_1:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.PICK_UP_1);
				break;
			case DRIVE_TO_SPEAKER:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.DRIVE_TO_SPEAKER);
				break;
			case PICK_UP_2:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.PICK_UP_2);
				break;
			case PICK_UP_3:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.PICK_UP_3);
				break;
			case LEAVE:
				isCurrentStateFinished = driveSystem.updateAutonomous(
					AutoFSMState.LEAVE);
				break;
			case SHOOTER_STATE:
				isCurrentStateFinished = shooterFSM.updateAutonomous(AutoFSMState.SHOOTER_STATE);
				break;
			case INTAKE_STATE:
				isCurrentStateFinished = shooterFSM.updateAutonomous(AutoFSMState.INTAKE_STATE);
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
