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
		OUTTAKING_AMP
	}

	private static final float AMP_L_MOTOR_RUN_POWER = 0.1f;
	private static final float AMP_U_MOTOR_RUN_POWER = -0.15f; // or -1.16f
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
				handleOuttakingSpeakerState(input);
				break;
			case OUTTAKING_AMP:
				handleOuttakingAmpState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		SmartDashboard.putString("Current State", currentState.toString());
		SmartDashboard.putBoolean("Bottom Limit Switch Pressed", bottomLimitSwitch.isPressed());
		SmartDashboard.putBoolean("Outtake Speaker Button Pressed",
			input.isOuttakeSpeakerButtonPressed());
		SmartDashboard.putBoolean("Outtake Amp Button Pressed",
			input.isOuttakeAmpButtonPressed());
		SmartDashboard.putBoolean("Intake Button Pressed", input.isIntakeButtonPressed());
		SmartDashboard.putNumber("Motor SPeed", highMotor.get());
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
				if (input.isOuttakeSpeakerButtonPressed() && !input.isIntakeButtonPressed()
					&& !input.isOuttakeAmpButtonPressed()) {
					outtakingTimerStart = timer.get();
					return ShooterFSMState.OUTTAKING_SPEAKER;
				}
				if (input.isOuttakeAmpButtonPressed() && !input.isIntakeButtonPressed()
					&& !input.isOuttakeSpeakerButtonPressed()) {
					outtakingTimerStart = timer.get();
					return ShooterFSMState.OUTTAKING_AMP;
				}
				if (input.isIntakeButtonPressed() && !hasNote()
					&& !input.isOuttakeSpeakerButtonPressed()
					&& !input.isOuttakeAmpButtonPressed()) {
					return ShooterFSMState.INTAKING;
				} else {
					return ShooterFSMState.IDLE_STOP;
				}
			case INTAKING:
				if (input.isIntakeButtonPressed() && !hasNote()
					&& !input.isOuttakeSpeakerButtonPressed()
					&& !input.isOuttakeAmpButtonPressed()) {
					return ShooterFSMState.INTAKING;
				} else {
					return ShooterFSMState.IDLE_STOP;
				}
			case OUTTAKING_SPEAKER:
				if (input.isOuttakeSpeakerButtonPressed() && !input.isIntakeButtonPressed()
					&& !input.isOuttakeAmpButtonPressed()) {
					return ShooterFSMState.OUTTAKING_SPEAKER;
				} else {
					return ShooterFSMState.IDLE_STOP;
				}
			case OUTTAKING_AMP:
				if (input.isOuttakeAmpButtonPressed() && !input.isIntakeButtonPressed()
					&& !input.isOuttakeSpeakerButtonPressed()) {
					return ShooterFSMState.OUTTAKING_AMP;
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
	private void handleOuttakingSpeakerState(TeleopInput input) {
		if (!timer.hasElapsed(outtakingTimerStart + OUTTAKING_TIME)) {
			highMotor.set(SPEAKER_U_MOTOR_RUN_POWER);
			if (timer.hasElapsed(outtakingTimerStart + (OUTTAKING_TIME / 2))) {
				lowMotor.set(SPEAKER_L_MOTOR_RUN_POWER);
			} else {
				lowMotor.set(0);
			}
		} else {
			highMotor.set(0);
			lowMotor.set(0);
		}
	}

	/**
	 * Handle behavior in OUTTAKING_AMP state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakingAmpState(TeleopInput input) {
		if (!timer.hasElapsed(outtakingTimerStart + OUTTAKING_TIME)) {
			highMotor.set(AMP_U_MOTOR_RUN_POWER);
			if (timer.hasElapsed(outtakingTimerStart + (OUTTAKING_TIME / 2))) {
				lowMotor.set(AMP_L_MOTOR_RUN_POWER);
			} else {
				lowMotor.set(0);
			}
		} else {
			highMotor.set(0);
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
