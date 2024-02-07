// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
// Systems
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.KitBotShooterFSM;
import frc.robot.systems.AutoHandlerSystem;
import frc.robot.systems.ClimberMechFSMLeft;
import frc.robot.systems.ClimberMechFSMRight;
import frc.robot.systems.AutoHandlerSystem.AutoPath;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	// Systems
	private DriveFSMSystem driveFSMSystem;
	private KitBotShooterFSM shooterFSM;
	private ClimberMechFSMLeft climberMechLeftFSM;
	private ClimberMechFSMRight climberMechRightFSM;
	private AutoHandlerSystem autoHandler;
	private AutoPathChooser autoPathChooser;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();
		// Instantiate all systems here
		autoPathChooser = new AutoPathChooser();
		driveFSMSystem = new DriveFSMSystem();
		//shooterFSM = new KitBotShooterFSM();
		//climberMechLeftFSM = new ClimberMechFSMLeft();
		//climberMechRightFSM = new ClimberMechFSMRight();
		autoHandler = new AutoHandlerSystem(driveFSMSystem);
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		AutoPath path = AutoPath.PATH1;
		if (AutoPathChooser.getSelectedPath() != null) {
			path = AutoPathChooser.getSelectedPath();
		}
		autoHandler.reset(path);
	}

	@Override
	public void autonomousPeriodic() {
		autoHandler.update();
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		driveFSMSystem.reset();
		//climberMechLeftFSM.reset();
		//climberMechRightFSM.reset();
		//shooterFSM.reset();
	}

	@Override
	public void teleopPeriodic() {
		driveFSMSystem.update(input);
		//climberMechLeftFSM.update(input);
		//climberMechRightFSM.update(input);
		//shooterFSM.update(input);
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

	}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
	}

	@Override
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }
}

