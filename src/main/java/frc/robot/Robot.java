// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;

// Systems
import frc.robot.systems.FSMSystem;
import frc.robot.systems.AutoHandlerSystem;
import frc.robot.systems.AutoHandlerSystem.AutoPath;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;

	// Systems
	// private FSMSystem subSystem1;
	// private FSMSystem subSystem2;
	// private FSMSystem subSystem3;

	// private AutoHandlerSystem autoHandler;
	private LED led;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();
		led = new LED(9);

		// Instantiate all systems here
		// subSystem1 = new FSMSystem();
		// subSystem2 = new FSMSystem();
		// subSystem3 = new FSMSystem();
		// autoHandler = new AutoHandlerSystem(subSystem1, subSystem2, subSystem3);
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		//autoHandler.reset(AutoPath.PATH1);
	}

	@Override
	public void autonomousPeriodic() {
		//led1.rainbow();
		led.chasingLED();
		//autoHandler.update();
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		// subSystem1.reset();
		// subSystem2.reset();
		// subSystem3.reset();
	}

	@Override
	public void teleopPeriodic() {
		// subSystem1.update(input);
		// subSystem2.update(input);
		// subSystem3.update(input);
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
	}

	@Override
	public void testPeriodic() {

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
