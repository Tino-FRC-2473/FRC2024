// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
// Systems
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.KitBotShooterFSM;
import frc.robot.systems.AutoHandlerSystem;
import frc.robot.systems.AutoHandlerSystem.AutoPath;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	// Systems
	private KitBotShooterFSM shooterFSM;
	private DriveFSMSystem driveFSMSystem;

	private AutoHandlerSystem autoHandler;
	private AutoPathChooser autoPathChooser;

	private UsbCamera driverCam;
	private UsbCamera chainCam;
	private VideoSink videoSink;
	private MjpegServer driverStream;
	private MjpegServer chainStream;

	private boolean chainCamToggled;

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
		shooterFSM = new KitBotShooterFSM();
		autoHandler = new AutoHandlerSystem(driveFSMSystem, shooterFSM);

		driverCam = CameraServer.startAutomaticCapture(0);
		chainCam = CameraServer.startAutomaticCapture(1);

		driverCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
		chainCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

		// videoSink = CameraServer.getServer();
		// chainCamToggled = false;
		// Creates the CvSource and MjpegServer [2] and connects them
		/*driverStream = CameraServer.putVideo("Driver Camera",
			VisionConstants.DRIVER_CAM_WIDTH_PIXELS, VisionConstants.DRIVER_CAM_HEIGHT_PIXELS);

		chainStream = CameraServer.putVideo("Chain Camera",
			VisionConstants.DRIVER_CAM_WIDTH_PIXELS, VisionConstants.DRIVER_CAM_HEIGHT_PIXELS);*/


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
		shooterFSM.reset();
	}

	@Override
	public void teleopPeriodic() {
		driveFSMSystem.update(input);
		shooterFSM.update(input);

		// if (input.chainChamToggleButton()) {
		// 	chainCamToggled = !chainCamToggled;
		// }

		// if (chainCamToggled) {
		// 	videoSink.setSource(chainCam);
		// } else {
		// 	videoSink.setSource(driverCam);
		// }

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
