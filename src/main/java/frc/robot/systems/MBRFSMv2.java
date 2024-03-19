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
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class MBRFSMv2 {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum MBRFSMState {
		MOVE_TO_GROUND,
		INTAKING,
		MOVE_TO_SHOOTER,
		SHOOTING,
		MOVE_TO_AMP,
	}

	private static final float SHOOTING_POWER = 0.8f;
	private static final double AUTO_SHOOTING_TIME = 1.0;

	private static final float INTAKE_POWER = 0.35f;
	private static final float OUTTAKE_POWER = -0.7f;
	private static final float HOLDING_POWER = 0.05f;
	private static final float AMP_SHOOT_POWER = -0.95f;
	private static final int AVERAGE_SIZE = 7;
	private static final float CURRENT_THRESHOLD = 11.0f;
	private double[] currLogs;
	private int tick = 0;
	private boolean holding = false;


	private static final double MIN_TURN_SPEED = -0.4;
	private static final double MAX_TURN_SPEED = 0.4;
	private static final double PID_CONSTANT_PIVOT_P = 0.0005;
	private static final double GROUND_ENCODER_ROTATIONS = -1200;
	private static final double AMP_ENCODER_ROTATIONS = -550;
	private static final double SHOOTER_ENCODER_ROTATIONS = 0;
	private static final double INRANGE_VALUE = 15;

	/* ======================== Private variables ======================== */
	private MBRFSMState currentState;
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
	public MBRFSMv2() {
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
		timer = new Timer();
		currLogs = new double[AVERAGE_SIZE];
		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public MBRFSMState getCurrentState() {
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
		currentState = MBRFSMState.MOVE_TO_SHOOTER;
		timer.stop();
		timer.reset();
		holding = false;
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

		currLogs[tick % AVERAGE_SIZE] = intakeMotor.getSupplyCurrent().getValueAsDouble();
		tick++;
		double avgcone = 0;
		for (int i = 0; i < AVERAGE_SIZE; i++) {
			avgcone += currLogs[i];
		}
		avgcone /= AVERAGE_SIZE;
		SmartDashboard.putBoolean("holding", holding);
		SmartDashboard.putNumber("avg current", avgcone);
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
			case MOVE_TO_AMP:
				handleMoveAmpState(input);
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
			case NOTE1:
				return handleAutoMoveGround() && handleAutoIntake();
			case NOTE2:
				return handleAutoMoveGround() && handleAutoIntake();
			case NOTE3:
				return handleAutoMoveGround() && handleAutoIntake();
			case NOTE4:
				return handleAutoMoveGround() && handleAutoIntake();
			case NOTE5:
				return handleAutoMoveGround() && handleAutoIntake();
			case NOTE6:
				return handleAutoMoveGround() && handleAutoIntake();
			case NOTE7:
				return handleAutoMoveGround() && handleAutoIntake();
			case NOTE8:
				return handleAutoMoveGround() && handleAutoIntake();
			case SPEAKER:
				return handleAutoMoveShooter() & handleAutoRev();
			case SHOOT:
				return handleAutoShoot();
			case SHOOT_PRELOADED:
				return handleAutoShootPreloaded();
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
	private MBRFSMState nextState(TeleopInput input) {
		switch (currentState) {
			case MOVE_TO_SHOOTER:
				if (input.isIntakeButtonPressed() && !input.isShootButtonPressed()
					&& !input.isRevButtonPressed() && !holding && !input.isAmpButtonPressed()) {
					return MBRFSMState.MOVE_TO_GROUND;
				}
				if (input.isAmpButtonPressed() && !input.isIntakeButtonPressed() && !input.isShootButtonPressed()
					&& !input.isRevButtonPressed()) {
					return MBRFSMState.MOVE_TO_AMP;
				}
				if (!input.isIntakeButtonPressed() && !input.isAmpButtonPressed() && (input.isShootButtonPressed()
					|| input.isRevButtonPressed())) {
					if (inRange(throughBore.getDistance(), SHOOTER_ENCODER_ROTATIONS)) {
						holding = false;
						return MBRFSMState.SHOOTING;
					} else {
						return MBRFSMState.MOVE_TO_SHOOTER;
					}
				}
				return MBRFSMState.MOVE_TO_SHOOTER;
			case MOVE_TO_GROUND:
				if (input.isIntakeButtonPressed() && !input.isShootButtonPressed()
					&& !input.isRevButtonPressed() && !input.isAmpButtonPressed()) {
					if (inRange(throughBore.getDistance(), GROUND_ENCODER_ROTATIONS)) {
						return MBRFSMState.INTAKING;
					} else {
						return MBRFSMState.MOVE_TO_GROUND;
					}
				}
				return MBRFSMState.MOVE_TO_SHOOTER;
			case INTAKING:
				if (input.isIntakeButtonPressed() && !input.isShootButtonPressed()
					&& !input.isRevButtonPressed() && !input.isAmpButtonPressed()) {
					if (hasNote()) {
						return MBRFSMState.MOVE_TO_SHOOTER;
					} else {
						return MBRFSMState.INTAKING;
					}
				}
				return MBRFSMState.MOVE_TO_SHOOTER;
			case SHOOTING:
				if (!input.isIntakeButtonPressed() && (input.isShootButtonPressed()
					|| input.isRevButtonPressed()) && !input.isAmpButtonPressed()) {
					return MBRFSMState.SHOOTING;
				}
				return MBRFSMState.MOVE_TO_SHOOTER;
			case MOVE_TO_AMP:
				if (input.isAmpButtonPressed() && !input.isShootButtonPressed()
					&& !input.isRevButtonPressed() && !input.isIntakeButtonPressed()) {
						return MBRFSMState.MOVE_TO_AMP;
				}
				return MBRFSMState.MOVE_TO_SHOOTER;
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
		if (input.isoverrideIntakeButtonPressed() && !input.isoverrideOuttakeButtonPressed()) {
			intakeMotor.set(INTAKE_POWER);
		} else if (!input.isoverrideIntakeButtonPressed() && input.isoverrideOuttakeButtonPressed()) {
			intakeMotor.set(OUTTAKE_POWER);
			holding = false;
		} else {
			intakeMotor.set(holding ? HOLDING_POWER : 0);
		}


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
		if (!holding) {
			intakeMotor.set(INTAKE_POWER);
		} else {
			intakeMotor.set(0);
		}
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

	public void handleMoveAmpState(TeleopInput input) {
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);
		pivotMotor.set(pid(throughBore.getDistance(), AMP_ENCODER_ROTATIONS));
		if (input.isShootAmpButtonPressed()) {
			holding = false;
			intakeMotor.set(AMP_SHOOT_POWER);
		} else {
			intakeMotor.set(0);
		}
	}
	/**
	 * Checks if the intake is holding a note.
	 * @return if the intake is holding a note
	 */
	public boolean hasNote() {
		double avgcone = 0;
		for (int i = 0; i < AVERAGE_SIZE; i++) {
			avgcone += currLogs[i];
		}
		avgcone /= AVERAGE_SIZE;
		if (avgcone > CURRENT_THRESHOLD) {
			holding = true;
		}
		return holding;
	}

	public boolean handleAutoMoveGround() {
		pivotMotor.set(pid(throughBore.getDistance(), GROUND_ENCODER_ROTATIONS));
		return inRange(throughBore.getDistance(), GROUND_ENCODER_ROTATIONS);
	}

	public boolean handleAutoMoveShooter() {
		intakeMotor.set(0);
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

	public boolean handleAutoShootPreloaded() {
		if (timer.get() == 0) {
			timer.start();
		}
		pivotMotor.set(pid(throughBore.getDistance(), SHOOTER_ENCODER_ROTATIONS));
		if (timer.get() < 1) {
			intakeMotor.set(0);
			shooterLeftMotor.set(-SHOOTING_POWER);
			shooterRightMotor.set(SHOOTING_POWER);
			return false;
		} else if (timer.get() < 2){
			intakeMotor.set(OUTTAKE_POWER);
			shooterLeftMotor.set(-SHOOTING_POWER);
			shooterRightMotor.set(SHOOTING_POWER);
			return false;
		} else {
			intakeMotor.set(0);
			shooterLeftMotor.set(0);
			shooterRightMotor.set(0);
			timer.stop();
			timer.reset();
			return true;
		}
	}

	public boolean handleAutoIntake() {
		intakeMotor.set(INTAKE_POWER + 0.3);
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);
		return true;
	}
}