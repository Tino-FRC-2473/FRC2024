// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Systems
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.MBRFSMv2;
import frc.robot.SwerveConstants.AutoConstants;
import frc.robot.systems.AutoHandlerSystem;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	// Systems
	private DriveFSMSystem driveFSMSystem;
	private MBRFSMv2 mechFSMSystem;
	private AutoHandlerSystem autoHandler;
	private AutoPathChooser autoPathChooser;

	private UsbCamera driverCam;

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
		mechFSMSystem = new MBRFSMv2();
		autoHandler = new AutoHandlerSystem(driveFSMSystem, mechFSMSystem);

		driverCam = CameraServer.startAutomaticCapture(0);
		driverCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		String path = "PROT";
		if (AutoPathChooser.getSelectedPath() != null) {
			path = AutoPathChooser.getSelectedPath();
		}
		String placement = "SWCT";
		if (AutoPathChooser.getSelectedPlacement() != null) {
			placement = AutoPathChooser.getSelectedPlacement();
		}
		String notes = "";
		for (int i = 0; i < AutoConstants.N_5; i++) {
			if (AutoPathChooser.getSelectedNote(i) != 0) {
				notes += AutoPathChooser.getSelectedNote(i);
			}
		}
		path += "_" + placement + "_" + notes;
		SmartDashboard.putString("AUTO PATH", path);
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
		mechFSMSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		driveFSMSystem.update(input);
		mechFSMSystem.update(input);
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
