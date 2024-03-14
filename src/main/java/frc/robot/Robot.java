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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Systems
import frc.robot.systems.IntakeFSM;
import frc.robot.systems.MBRShooterFSM;
import frc.robot.systems.PivotFSM;
import frc.robot.systems.DriveFSMSystem;
// import frc.robot.systems.KitBotShooterFSM;
import frc.robot.SwerveConstants.AutoConstants;
import frc.robot.systems.AutoHandlerSystem;
// import frc.robot.systems.ClimberMechFSMLeft;
// import frc.robot.systems.ClimberMechFSMRight;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	// Systems
	private IntakeFSM intakeFSM;
	private MBRShooterFSM shooterFSM;
	private PivotFSM pivotFSM;
	// private KitBotShooterFSM shooterFSM;
	// private ClimberMechFSMLeft climberMechLeftFSM;
	// private ClimberMechFSMRight climberMechRightFSM;
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
		//intakeFSM = new IntakeFSM();
		//shooterFSM = new MBRShooterFSM();
		pivotFSM = new PivotFSM();
		autoHandler = new AutoHandlerSystem(driveFSMSystem, intakeFSM, shooterFSM, pivotFSM);
		autoPathChooser = new AutoPathChooser();
		driveFSMSystem = new DriveFSMSystem();
		// shooterFSM = new KitBotShooterFSM();
		// climberMechLeftFSM = new ClimberMechFSMLeft();
		// climberMechRightFSM = new ClimberMechFSMRight();

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
		intakeFSM.reset();
		shooterFSM.reset();
		pivotFSM.reset();
		driveFSMSystem.reset();
		// climberMechLeftFSM.reset();
		// climberMechRightFSM.reset();
		// shooterFSM.reset();
	}

	@Override
	public void teleopPeriodic() {
		intakeFSM.update(input);
		shooterFSM.update(input);
		pivotFSM.update(input);
		driveFSMSystem.update(input);
	//	climberMechLeftFSM.update(input);
		// climberMechRightFSM.update(input);
		// shooterFSM.update(input);

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
