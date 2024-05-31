// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.util.PixelFormat;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.SwerveConstants.AutoConstants;
import frc.robot.SwerveConstants.DriveConstants;

// Systems
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.MBRFSMv2;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	// Systems
	private DriveFSMSystem driveFSMSystem;
	private MBRFSMv2 mbrfsMv2;
	SendableChooser<Command> autoChooser;
	Command autonomousCommand;
	private final Field2d m_field = new Field2d();

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();
		driveFSMSystem = new DriveFSMSystem();
		mbrfsMv2 = new MBRFSMv2();
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
		SmartDashboard.putData("Field", m_field);
		// NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
		// NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
		// NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());

		// Instantiate all systems here
	}


	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		driveFSMSystem.resetAutonomus();
		autonomousCommand = getAutonomousCommand();
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
		// driveFSMSystem.updateAutonomous();
		m_field.setRobotPose(driveFSMSystem.getPose());
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		driveFSMSystem.reset();
		mbrfsMv2.reset();
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		driveFSMSystem.update(input);
		mbrfsMv2.update(input);
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

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
