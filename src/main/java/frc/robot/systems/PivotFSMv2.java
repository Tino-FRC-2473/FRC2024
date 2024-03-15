package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
//import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

public class PivotFSMv2 {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum PivotFSMState {
		MOVE_TO_GROUND,
		INTAKING,
		MOVE_TO_SHOOTER,
		SHOOTING
	}

	private static final float SHOOTING_POWER = 0.7f;

	private static final float INTAKE_POWER = 0.1f;
	private static final float OUTTAKE_POWER = -0.1f;
	private static final float HOLDING_POWER = 0.03f;
	private static final int AVERAGE_SIZE = 7;

	private static final double MIN_TURN_SPEED = -0.15;
	private static final double MAX_TURN_SPEED = 0.15;
	private static final double PID_CONSTANT_PIVOT_P = 0.001;
	private static final double GROUND_ENCODER_ROTATIONS = -1000;
	private static final double SHOOTER_ENCODER_ROTATIONS = 0;
	private static final double INRANGE_VALUE = 5;

	/* ======================== Private variables ======================== */
	private PivotFSMState currentState;
	private CANSparkMax shooterLeftMotor;
	private CANSparkMax shooterRightMotor;
	private TalonFX intakeMotor;
	private TalonFX pivotMotor;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	private double currentEncoder;
	private Encoder throughBore;




	/* ======================== Constructor ======================== */
	/**
	 * Create PivotFSM and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public PivotFSMv2() {
		shooterLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_LSHOOTER_MOTOR,
										CANSparkMax.MotorType.kBrushless);

		shooterRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_RSHOOTER_MOTOR,
										CANSparkMax.MotorType.kBrushless);
		intakeMotor = new TalonFX(HardwareMap.DEVICE_ID_INTAKE_MOTOR);
		intakeMotor.setNeutralMode(NeutralModeValue.Brake);

		pivotMotor = new TalonFX(HardwareMap.DEVICE_ID_ARM_MOTOR);
		pivotMotor.setNeutralMode(NeutralModeValue.Brake);

		currentEncoder = 0;
		throughBore = new Encoder(0, 1);
		throughBore.reset();

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public PivotFSMState getCurrentState() {
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
		currentState = PivotFSMState.MOVE_TO_SHOOTER;

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
			case MOVE_TO_SHOOTER:
				handleMoveShooterState(input);
				break;
			case INTAKING:
				handleIntakingState(input);
				break;
			case MOVE_TO_GROUND:
				handleMoveGroundState(input);
				break;
			case SHOOTING:
				handleShootingState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);
	}

	/**
	 * Performs specific action based on the autoState passed in.
	 * @param autoState autoState that the subsystem executes.
	 * @return if the action carried out in this state has finished executing
	 */
	public boolean updateAutonomous(AutoFSMState autoState) {
		switch (autoState) {
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
	private PivotFSMState nextState(TeleopInput input) {
		switch (currentState) {
			case MOVE_TO_SHOOTER:
				if (input.isIntakeButtonPressed() && !input.isShootButtonPressed()
					&& !input.isRevButtonPressed()) {
					return PivotFSMState.MOVE_TO_GROUND;
				}
				if (!input.isIntakeButtonPressed() && (input.isShootButtonPressed()
					|| input.isRevButtonPressed())) {
					return PivotFSMState.SHOOTING;
				}
				return PivotFSMState.MOVE_TO_SHOOTER;
			case MOVE_TO_GROUND:
				if (input.isIntakeButtonPressed() && !input.isShootButtonPressed()
					&& !input.isRevButtonPressed()) {
					if (inRange(throughBore.getDistance(), GROUND_ENCODER_ROTATIONS)) {
						return PivotFSMState.INTAKING;
					} else {
						return PivotFSMState.MOVE_TO_GROUND;
					}
				}
				return PivotFSMState.MOVE_TO_SHOOTER;
			case INTAKING:
				if (input.isIntakeButtonPressed() && !input.isShootButtonPressed()
					&& !input.isRevButtonPressed()) {
					if (hasNote()) {
						return PivotFSMState.MOVE_TO_SHOOTER;
					} else {
						return PivotFSMState.INTAKING;
					}
				}
				return PivotFSMState.MOVE_TO_SHOOTER;
			case SHOOTING:
				if (!input.isIntakeButtonPressed() && (input.isShootButtonPressed()
					|| input.isRevButtonPressed())) {
					return PivotFSMState.SHOOTING;
				}
				return PivotFSMState.MOVE_TO_SHOOTER;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */

	private boolean inRange(double a, double b) {
		return Math.abs(a - b) < INRANGE_VALUE; //EXPERIMENTAL
	}

	private double pid(double currentEncoderPID, double targetEncoder) {
		double correction = PID_CONSTANT_PIVOT_P * (targetEncoder - currentEncoderPID);
		return Math.min(MAX_TURN_SPEED, Math.max(MIN_TURN_SPEED, correction));
	}

	public void handleMoveShooterState(TeleopInput input) {
		pivotMotor.set(pid(throughBore.getDistance(), SHOOTER_ENCODER_ROTATIONS));
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);
		intakeMotor.set(0);
	}
	public void handleMoveGroundState(TeleopInput input) {
		pivotMotor.set(pid(throughBore.getDistance(), GROUND_ENCODER_ROTATIONS));
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);
		intakeMotor.set(0);
	}
	public void handleIntakingState(TeleopInput input) {
		pivotMotor.set(pid(throughBore.getDistance(), GROUND_ENCODER_ROTATIONS));
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);
		intakeMotor.set(INTAKE_POWER);
	}
	public void handleShootingState(TeleopInput input) {
		pivotMotor.set(pid(throughBore.getDistance(), SHOOTER_ENCODER_ROTATIONS));
		if (input.isRevButtonPressed() && !input.isShootButtonPressed()) {
			shooterLeftMotor.set(SHOOTING_POWER);
			shooterRightMotor.set(SHOOTING_POWER);
			intakeMotor.set(0);
		}
		if (input.isShootButtonPressed()) {
			shooterLeftMotor.set(SHOOTING_POWER);
			shooterRightMotor.set(SHOOTING_POWER);
			intakeMotor.set(OUTTAKE_POWER);
		}
	}

	public boolean hasNote() {
		return false;
	}
}
