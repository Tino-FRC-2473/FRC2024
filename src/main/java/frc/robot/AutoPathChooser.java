package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.AutoHandlerSystem.AutoPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoPathChooser {
	private static SendableChooser<AutoPath> autoPathChooser;
	private static SendableChooser<Boolean> allianceChooser;
	private static SendableChooser<Integer> startingPosChooser;
	private static SendableChooser<Boolean> mechChooser;


	/**
	 * Constructor for creating the AutoPathChooser class. It is used in order to select the
	 * auto path and node for the robot without re-deploying.
	 */
	public AutoPathChooser() {
		autoPathChooser = new SendableChooser<>();
		autoPathChooser.setDefaultOption("Score and Leave", AutoPath.PATH1);
		autoPathChooser.addOption("MBR ONLY: Score Multiple Times", AutoPath.PATH2);
		autoPathChooser.addOption("Score Only", AutoPath.PATH3);
		SmartDashboard.putData("Auto Path", autoPathChooser);

		allianceChooser = new SendableChooser<>();
		allianceChooser.setDefaultOption("Blue", true);
		allianceChooser.addOption("Red", false);
		SmartDashboard.putData("Alliance", allianceChooser);

		startingPosChooser = new SendableChooser<>();
		startingPosChooser.setDefaultOption("Speaker (center)", 0);
		startingPosChooser.addOption("Speaker (source side)", 1);
		startingPosChooser.addOption("Speaker (amp side)", 2);
		startingPosChooser.addOption("Amp", 2 + 1);
		startingPosChooser.addOption("Other", 2 + 2);
		SmartDashboard.putData("Starting Position", startingPosChooser);

		mechChooser = new SendableChooser<>();
		mechChooser.setDefaultOption("SVR Mech", true);
		mechChooser.addOption("MBR Mech", false);
		SmartDashboard.putData("Mech", mechChooser);
	}

	/**
	 * Returns the sendable chooser object which contains information on the selected auto path.
	 * @return The sendable chooser for the auto path. SendableChooser contains AutoPath.
	 */
	public static SendableChooser<AutoPath> getAutoPathChooser() {
		return autoPathChooser;
	}

	/**
	 * Returns the sendable chooser object containing information on the selected alliance.
	 * @return the sendable chooser for the alliance. SendableChooser contains Boolean.
	 */
	public static SendableChooser<Boolean> getAllianceChooser() {
		return allianceChooser;
	}

	/**
	 * Returns the sendable chooser object containing information on the selected starting position.
	 * @return the sendable chooser for the starting position. SendableChooser contains Integer.
	 */
	public static SendableChooser<Integer> getStartPosChooser() {
		return startingPosChooser;
	}

	/**
	 * Returns the sendable chooser object containing information on the selected mechanism.
	 * @return the sendable chooser for the mechanism. SendableChooser contains Boolean.
	 */
	public static SendableChooser<Boolean> getMechChooser() {
		return mechChooser;
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
	 * @return boolean for the selected alliance.
	 */
	public static boolean getSelectedAlliance() {
		return allianceChooser.getSelected();
	}

	/**
	 * Returns the selected starting position.
	 * @return integer for the selected starting position.
	 */
	public static int getStartingPos() {
		return startingPosChooser.getSelected();
	}

	/**
	 * Returns the selected mechanism.
	 * @return boolean for the selected mechanism.
	 */
	public static boolean getSelectedMech() {
		return mechChooser.getSelected();
	}
}
