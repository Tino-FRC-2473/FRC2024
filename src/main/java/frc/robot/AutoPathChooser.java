package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.AutoHandlerSystem.AutoPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoPathChooser {
	private static SendableChooser<AutoPath> autoPathChooser;
	private static SendableChooser<Boolean> allianceChooser;
	private static SendableChooser<Integer> startingPosChooser;


	/**
	 * Constructor for creating the AutoPathChooser class. It is used in order to select the
	 * auto path and node for the robot without re-deploying.
	 */
	public AutoPathChooser() {
		autoPathChooser = new SendableChooser<>();
		autoPathChooser.setDefaultOption("Score in Speaker Twice", AutoPath.PATH1);
		autoPathChooser.addOption("Score in Amp Twice", AutoPath.PATH2);
		autoPathChooser.addOption("Shoot Speaker and Leave", AutoPath.PATH3);
		autoPathChooser.addOption("Score Amp and Leave", AutoPath.PATH4);
		autoPathChooser.addOption("Leave Only", AutoPath.PATH5);
		autoPathChooser.addOption("Shoot Only", AutoPath.PATH6);
		SmartDashboard.putData("Auto Path", autoPathChooser);

		allianceChooser = new SendableChooser<>();
		allianceChooser.setDefaultOption("Blue", true);
		allianceChooser.addOption("Red", false);
		SmartDashboard.putData("Alliance", allianceChooser);

		startingPosChooser = new SendableChooser<>();
		startingPosChooser.setDefaultOption("Center", 1);
		startingPosChooser.addOption("Left", 2);
		startingPosChooser.addOption("Right", 3);
		SmartDashboard.putData("Starting Position", startingPosChooser);
	}

	/**
	 * Returns the sendable chooser object which contains information on the selected auto path.
	 * @return The sendable chooser for the auto path.SendableChooser contains FSMState.
	 */
	public static SendableChooser<AutoPath> getAutoPathChooser() {
		return autoPathChooser;
	}

	/**
	 * Returns the sendable chooser object containing information on the selected auto path
     * alliance.
	 * @return the sendable chooser for the alliance. SendableChooser contains Boolean.
	 */
	public static SendableChooser<Boolean> getAllianceChooser() {
		return allianceChooser;
	}

	/**
	 * Returns the sendable chooser object containing information on the selected starting position.
	 * @return the sendable chooser for the alliance. SendableChooser contains Boolean.
	 */
	public static SendableChooser<Integer> getStartPosChooser() {
		return startingPosChooser;
	}

	/**
	 * Returs the selected auto path.
	 * @return FSMState object which has the selected auto path.
	 */
	public static AutoPath getSelectedPath() {
		return autoPathChooser.getSelected();
	}

	/**
	 * Returns the selected alliance.
	 * @return boolean for the selected node. true for blue, false for red.
	 */
	public static boolean getSelectedAlliance() {
		return allianceChooser.getSelected();
	}

	/**
	 * Returns the selected starting position.
	 * @return boolean for the selected node. true for blue, false for red.
	 */
	public static int getStartingPos() {
		return startingPosChooser.getSelected();
	}
}
