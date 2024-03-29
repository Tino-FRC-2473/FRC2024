package frc.robot.systems;

import edu.wpi.first.wpilibj.Timer;

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
		OUTTAKING_SPEAKER,
	}

	private static final float SPEAKER_L_MOTOR_RUN_POWER = 0.8f;
	private static final float SPEAKER_U_MOTOR_RUN_POWER = -1.0f;
	private static final float INTAKING_SPEED = -0.4f;
	private static final float OUTTAKING_TIME = 2.0f;

	/* ======================== Private variables ======================== */
	private ShooterFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax lowMotor;
	private CANSparkMax highMotor;
	private SparkLimitSwitch bottomLimitSwitch;

	private boolean autoOuttakingTimerStarted;
	private double outtakingTimerStart;
	private Timer timer = new Timer();

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public KitBotShooterFSM() {
		// Perform hardware init
		lowMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER_LOWER,
						CANSparkMax.MotorType.kBrushless);
		lowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		bottomLimitSwitch = lowMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
		bottomLimitSwitch.enableLimitSwitch(false);

		highMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER_UPPER,
		CANSparkMax.MotorType.kBrushless);
		highMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		autoOuttakingTimerStarted = false;

		timer = new Timer();

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
		timer = new Timer();
		timer.start();
		autoOuttakingTimerStarted = false;
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
				handleIdleState(input);
				break;
			case INTAKING:
				handleIntakingState(input);
				break;
			case OUTTAKING_SPEAKER:
				handleShootSpeakerState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		SmartDashboard.putString("Current State", currentState.toString());
		SmartDashboard.putBoolean("Bottom Limit Switch Pressed", bottomLimitSwitch.isPressed());
		SmartDashboard.putBoolean("Intake Button Pressed", input.isIntakeButtonPressed());
		SmartDashboard.putNumber("Motor Speed", highMotor.get());
		SmartDashboard.putNumber("Get applied output", highMotor.getAppliedOutput());
		currentState = nextState(input);
	}

	/**
	 * Performs specific action based on the autoState passed in.
	 * @param autoState autoState that the subsystem executes.
	 * @return if the action carried out in this state has finished executing
	 */
	public boolean updateAutonomous(AutoFSMState autoState) {
		switch (autoState) {
			case SHOOTER_STATE:
				return handleAutoOuttakingState();
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
				if ((input.isShootButtonPressed() || input.isRevOuttakeButtonPressed())
					&& !input.isIntakeButtonPressed()) {
					return ShooterFSMState.OUTTAKING_SPEAKER;
				}
				if (input.isIntakeButtonPressed() && !hasNote()
					&& !input.isShootButtonPressed()
					&& !input.isRevOuttakeButtonPressed()) {
					return ShooterFSMState.INTAKING;
				}
				return ShooterFSMState.IDLE_STOP;
			case INTAKING:
				if (input.isIntakeButtonPressed() && !hasNote()
					&& !input.isShootButtonPressed()
					&& !input.isRevOuttakeButtonPressed()) {
					return ShooterFSMState.INTAKING;
				} else {
					return ShooterFSMState.IDLE_STOP;
				}
			case OUTTAKING_SPEAKER:
				if ((input.isShootButtonPressed() || input.isRevOuttakeButtonPressed())
					&& !input.isIntakeButtonPressed()) {
					return ShooterFSMState.OUTTAKING_SPEAKER;
				} else {
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
	private void handleIdleState(TeleopInput input) {
		lowMotor.set(0);
		highMotor.set(0);
	}
	/**
	 * Handle behavior in INTAKING state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakingState(TeleopInput input) {
		lowMotor.set(INTAKING_SPEED);
		highMotor.set(-INTAKING_SPEED);
	}

	/**
	 * Handle behavior in OUTTAKING_SPEAKER state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleShootSpeakerState(TeleopInput input) {
		if (input.isRevOuttakeButtonPressed()) {
			highMotor.set(SPEAKER_U_MOTOR_RUN_POWER);
		} else {
			highMotor.set(0);
		}
		if (input.isShootButtonPressed()) {
			lowMotor.set(SPEAKER_L_MOTOR_RUN_POWER);
		} else {
			lowMotor.set(0);
		}
	}

	private boolean handleAutoOuttakingState() {
		if (!autoOuttakingTimerStarted) {
			autoOuttakingTimerStarted = true;
			outtakingTimerStart = timer.get();
		}
		if (autoOuttakingTimerStarted
			&& !timer.hasElapsed(outtakingTimerStart + OUTTAKING_TIME)) {
			highMotor.set(SPEAKER_U_MOTOR_RUN_POWER);
			if (timer.hasElapsed(outtakingTimerStart + (OUTTAKING_TIME / 2))) {
				lowMotor.set(SPEAKER_L_MOTOR_RUN_POWER);
			} else {
				lowMotor.set(0);
			}

		} else {
			lowMotor.set(0);
			highMotor.set(0);

			autoOuttakingTimerStarted = false;
			return true;
		}

		return false;
	}

	private boolean hasNote() {
		return bottomLimitSwitch.isPressed();
	}

}
