package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// WPILib Imports
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
//import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Encoder;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class PivotFSMv2 {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum PivotFSMState {
		MOVE_TO_GROUND,
		INTAKING,
		MOVE_TO_SHOOTER,
		SHOOTING
	}

	public enum AutoFSMState {
		SPEAKER,
		NOTE,
		SHOOT
	}

	private static final float SHOOTING_POWER = 0.5f;
	private static final double AUTO_SHOOTING_TIME = 1.0;

	private static final float INTAKE_POWER = 0.1f;
	private static final float OUTTAKE_POWER = -0.1f;
	private static final float HOLDING_POWER = 0.03f;
	private static final int AVERAGE_SIZE = 7;
	private static final float CURRENT_THRESHOLD = 15.0f;
	private double[] currLogs;
	private int tick = 0;
	private boolean holding = false;


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

	private Encoder throughBore;
	private Timer timer;



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

		throughBore = new Encoder(0, 1);
		throughBore.reset();

		currLogs = new double[AVERAGE_SIZE];
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
		timer.stop();
		timer.reset();

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

		currLogs[tick % AVERAGE_SIZE] = intakeMotor.getTorqueCurrent().getValueAsDouble();
		tick++;
		SmartDashboard.putString("Current State", getCurrentState().toString());
		SmartDashboard.putNumber("Intake power", intakeMotor.get());
		SmartDashboard.putNumber("Pivot power", pivotMotor.get());
		SmartDashboard.putNumber("Left shooter power", shooterLeftMotor.get());
		SmartDashboard.putNumber("Right shooter power", shooterRightMotor.get());
		SmartDashboard.putNumber("Pivot encoder count", throughBore.getDistance());

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
			case NOTE:
				return handleAutoMoveGround() && handleAutoIntake();
			case SPEAKER:
				return handleAutoMoveShooter() && handleAutoRev();
			case SHOOT:
				return handleAutoShoot();
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
					if (inRange(throughBore.getDistance(), SHOOTER_ENCODER_ROTATIONS)) {
						return PivotFSMState.SHOOTING;
					} else {
						return PivotFSMState.MOVE_TO_SHOOTER;
					}
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

	/**
	 * Handles the moving to shooter state of the MBR Mech.
	 * @param input
	 */
	public void handleMoveShooterState(TeleopInput input) {
		pivotMotor.set(pid(throughBore.getDistance(), SHOOTER_ENCODER_ROTATIONS));
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);
		intakeMotor.set(0);
	}

	/**
	 * Handles the moving to ground state of the MBR Mech.
	 * @param input
	 */
	public void handleMoveGroundState(TeleopInput input) {
		pivotMotor.set(pid(throughBore.getDistance(), GROUND_ENCODER_ROTATIONS));
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);
		intakeMotor.set(0);
	}

	/**
	 * Handles the intaking state of the MBR Mech.
	 * @param input
	 */
	public void handleIntakingState(TeleopInput input) {
		pivotMotor.set(pid(throughBore.getDistance(), GROUND_ENCODER_ROTATIONS));
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);
		intakeMotor.set(INTAKE_POWER);
	}

	/**
	 * Handles the shooting state of the MBR Mech.
	 * @param input
	 */
	public void handleShootingState(TeleopInput input) {
		pivotMotor.set(pid(throughBore.getDistance(), SHOOTER_ENCODER_ROTATIONS));
		if (input.isRevButtonPressed() && !input.isShootButtonPressed()) {
			shooterLeftMotor.set(-SHOOTING_POWER);
			shooterRightMotor.set(SHOOTING_POWER);
			intakeMotor.set(0);
		}
		if (input.isShootButtonPressed()) {
			shooterLeftMotor.set(-SHOOTING_POWER);
			shooterRightMotor.set(SHOOTING_POWER);
			intakeMotor.set(OUTTAKE_POWER);
		}
	}

	/**
	 * Checks if the intake is holding a note.
	 * NEEDS TO BE IMPLEMENTED STILL
	 * @return if the intake is holding a note
	 */
	public boolean hasNote() {
		double avgcone = 0;
		for (int i = 0; i < AVERAGE_SIZE; i++) {
			avgcone += currLogs[i];
		}
		avgcone /= AVERAGE_SIZE;
		SmartDashboard.putNumber("avg current", avgcone);
		return false;
	}

	public boolean handleAutoMoveGround() {
		pivotMotor.set(pid(throughBore.getDistance(), GROUND_ENCODER_ROTATIONS));
		return inRange(throughBore.getDistance(), GROUND_ENCODER_ROTATIONS);
	}

	public boolean handleAutoMoveShooter() {
		intakeMotor.set(HOLDING_POWER);
		pivotMotor.set(pid(throughBore.getDistance(), SHOOTER_ENCODER_ROTATIONS));
		return inRange(throughBore.getDistance(), SHOOTER_ENCODER_ROTATIONS);
	}

	public boolean handleAutoRev() {
		shooterLeftMotor.set(-SHOOTING_POWER);
		shooterRightMotor.set(SHOOTING_POWER);
		return true;
	}

	public boolean handleAutoShoot() {
		if (timer.get() == 0) {
			timer.start();
		}
		pivotMotor.set(pid(throughBore.getDistance(), SHOOTER_ENCODER_ROTATIONS));
		if (timer.get() > AUTO_SHOOTING_TIME) {
			intakeMotor.set(0);
			shooterLeftMotor.set(0);
			shooterRightMotor.set(0);
			timer.stop();
			timer.reset();
			return true;
		} else {
			intakeMotor.set(OUTTAKE_POWER);
			shooterLeftMotor.set(-SHOOTING_POWER);
			shooterRightMotor.set(SHOOTING_POWER);
			return false;
		}
	}

	public boolean handleAutoIntake() {
		intakeMotor.set(INTAKE_POWER);
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);
		return true;
	}
}
