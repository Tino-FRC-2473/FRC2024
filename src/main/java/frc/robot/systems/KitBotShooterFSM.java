package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

public class KitBotShooterFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum ShooterFSMState {
		IDLE_STOP,
		INTAKING,
		OUTTAKING
	}

	private static final float L_MOTOR_RUN_POWER = 0.05f;
	private static final float U_MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private ShooterFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax lowMotor;
	private CANSparkMax highMotor;
	private SparkLimitSwitch bottomLimitSwitch;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public KitBotShooterFSM() {
		// Perform hardware init
		// [Initialize lowMotor and highMotor]
		//exampleMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
		//CANSparkMax.MotorType.kBrushless);

		lowMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER_LOWER,
						CANSparkMax.MotorType.kBrushless);
		lowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		lowMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
		bottomLimitSwitch.enableLimitSwitch(false);

		highMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER_UPPER,
		CANSparkMax.MotorType.kBrushless);
		highMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public ShooterFSMState getCurrentState() {
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
		currentState = ShooterFSMState.IDLE_STOP;

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

		switch (currentState) {
			case IDLE_STOP:
				handleStartState(input);
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
		SmartDashboard.putString("Current State", currentState.toString());
		SmartDashboard.putBoolean("Bottom Limit Switch Pressed", bottomLimitSwitch.isPressed());
		SmartDashboard.putBoolean("Outtake Button Pressed", input.isOuttakeButtonPressed());
		SmartDashboard.putBoolean("Intake Button Pressed", input.isIntakeButtonPressed());
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
	private ShooterFSMState nextState(TeleopInput input) {
		if (input == null) {
			return ShooterFSMState.IDLE_STOP;
		}
		switch (currentState) {
			case IDLE_STOP:
				if (input.isOuttakeButtonPressed() && !input.isIntakeButtonPressed()) {
					return ShooterFSMState.OUTTAKING;
				}
				if (input.isIntakeButtonPressed() && !hasNote()
					&& !input.isOuttakeButtonPressed()) {
					return ShooterFSMState.INTAKING;
				}
				if ((input.isIntakeButtonPressed() && hasNote())
					|| !(input.isOuttakeButtonPressed() || input.isIntakeButtonPressed())
					|| (input.isIntakeButtonPressed() && input.isOuttakeButtonPressed())) {
					return ShooterFSMState.IDLE_STOP;
				}
			case INTAKING:
				if (input.isIntakeButtonPressed() && !hasNote()
					&& !input.isOuttakeButtonPressed()) {
					return ShooterFSMState.INTAKING;
				}
				if (!input.isIntakeButtonPressed() || hasNote()
					|| (input.isIntakeButtonPressed() && input.isOuttakeButtonPressed())) {
					return ShooterFSMState.IDLE_STOP;
				}
			case OUTTAKING:
				if (input.isOuttakeButtonPressed() && !input.isIntakeButtonPressed()) {
					return ShooterFSMState.OUTTAKING;
				}
				if (!input.isOuttakeButtonPressed()
					|| (input.isIntakeButtonPressed() && input.isOuttakeButtonPressed())) {
					return ShooterFSMState.IDLE_STOP;
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
	private void handleStartState(TeleopInput input) {
		lowMotor.set(0);
		highMotor.set(0);
	}
	/**
	 * Handle behavior in INTAKING state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakingState(TeleopInput input) {
		lowMotor.set(-L_MOTOR_RUN_POWER);
		highMotor.set(-U_MOTOR_RUN_POWER);
	}

	/**
	 * Handle behavior in OUTTAKING state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakingState(TeleopInput input) {
		lowMotor.set(L_MOTOR_RUN_POWER);
		highMotor.set(U_MOTOR_RUN_POWER);
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

	private boolean hasNote() {
		return bottomLimitSwitch.isPressed();
	}



}
