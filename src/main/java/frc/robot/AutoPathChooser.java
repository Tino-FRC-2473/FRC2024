package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveConstants.AutoConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.ArrayList;

public class AutoPathChooser {
	/* --------------------------- SVR --------------------------- */
	// private static SendableChooser<AutoPath> autoPathChooser;
	// private static SendableChooser<Integer> startingPosChooser;
	// private static SendableChooser<Boolean> mechChooser;
	/* --------------------------- SVR --------------------------- */

	private static SendableChooser<Boolean> allianceChooser;
	private static SendableChooser<String> pathChooser;
	private static SendableChooser<String> placementChooser;
	private static ArrayList<SendableChooser<Integer>> noteChoosers;

	/**
	 * Constructor for creating the AutoPathChooser class. It is used in order to select the
	 * auto path and node for the robot without re-deploying.
	 */
	public AutoPathChooser() {
		/* --------------------------- SVR --------------------------- */
		// autoPathChooser = new SendableChooser<>();
		// autoPathChooser.setDefaultOption("Score and Leave", AutoPath.PATH1);
		// autoPathChooser.addOption("MBR ONLY: Score Multiple Times", AutoPath.PATH2);
		// autoPathChooser.addOption("Score Only", AutoPath.PATH3);
		// autoPathChooser.addOption("Auto Destroyer", AutoPath.PATH4);
		// autoPathChooser.addOption("No Shoot Auto Destroyer", AutoPath.PATH5);
		// SmartDashboard.putData("Auto Path", autoPathChooser);

		// startingPosChooser = new SendableChooser<>();
		// startingPosChooser.setDefaultOption("Speaker (center)", 0);
		// startingPosChooser.addOption("Speaker (source side)", 1);
		// startingPosChooser.addOption("Speaker (amp side)", 2);
		// startingPosChooser.addOption("Amp", 2 + 1);
		// startingPosChooser.addOption("Other", 2 + 2);
		// SmartDashboard.putData("Starting Position", startingPosChooser);

		// mechChooser = new SendableChooser<>();
		// mechChooser.setDefaultOption("SVR Mech", true);
		// mechChooser.addOption("MBR Mech", false);
		// SmartDashboard.putData("Mech", mechChooser);
		/* --------------------------- SVR --------------------------- */

		allianceChooser = new SendableChooser<>();
		allianceChooser.setDefaultOption("Blue", true);
		allianceChooser.addOption("Red", false);
		SmartDashboard.putData("Alliance", allianceChooser);

		pathChooser = new SendableChooser<>();
		pathChooser.setDefaultOption("Target Protected", "PROT");
		pathChooser.addOption("Target Miscellaneous", "MISC");
		pathChooser.addOption("Target Midfield", "MIDF");
		pathChooser.addOption("AUTO DESTROYER", "AUTO");
		pathChooser.addOption("Safety Path", "SAFE");
		SmartDashboard.putData("Path Chooser", pathChooser);

		placementChooser = new SendableChooser<>();
		placementChooser.setDefaultOption("Subwoofer Center", "SWCT");
		placementChooser.addOption("Subwoofer Source", "SWSR");
		placementChooser.addOption("Subwoofer Amp", "SWAM");
		placementChooser.addOption("Source", "BYSR");
		placementChooser.addOption("Amp", "BYAM");
		SmartDashboard.putData("Starting Position", placementChooser);

		noteChoosers = new ArrayList<>();
		String[] key = {"First ", "Second ", "Third ", "Fourth ", "Fifth "};
		for (int i = 0; i < key.length; i++) {
			SendableChooser<Integer> noteChooser = new SendableChooser<>();
			if (noteChooser != null) {
				noteChooser.setDefaultOption("N/A", 0);
				for (int j = 1; j <= AutoConstants.N_8; j++) {
					noteChooser.addOption("Note " + j, j);
				}
				SmartDashboard.putData(key[i] + "Note", noteChooser);
				noteChoosers.add(noteChooser);
			}
		}
	}

	/* --------------------------- SVR --------------------------- */
	// /**
	//  * Returns the sendable chooser object which contains information on the selected auto path.
	//  * @return The sendable chooser for the auto path. SendableChooser contains AutoPath.
	//  */
	// public static SendableChooser<AutoPath> getAutoPathChooser() {
	// 	return autoPathChooser;
	// }

	// /**
	//  * Returns the sendable chooser object containing information on the selected starting pos.
	//  * @return the sendable chooser for the starting position. SendableChooser contains Integer.
	//  */
	// public static SendableChooser<Integer> getStartPosChooser() {
	// 	return startingPosChooser;
	// }

	// /**
	//  * Returns the sendable chooser object containing information on the selected mechanism.
	//  * @return the sendable chooser for the mechanism. SendableChooser contains Boolean.
	//  */
	// public static SendableChooser<Boolean> getMechChooser() {
	// 	return mechChooser;
	// }
	/* --------------------------- SVR --------------------------- */

	/**
	 * Returns the sendable chooser object containing information on the selected alliance.
	 * @return the sendable chooser for the alliance. SendableChooser contains Boolean.
	 */
	public static SendableChooser<Boolean> getAllianceChooser() {
		return allianceChooser;
	}

	/**
	 * Returns the sendable chooser object containing information on the selected path.
	 * @return the sendable chooser for the path. SendableChooser contains String.
	 */
	public static SendableChooser<String> getPathChooser() {
		return pathChooser;
	}

	/**
	 * Returns the sendable chooser object containing information on the selected placement.
	 * @return the sendable chooser for the placement. SendableChooser contains String.
	 */
	public static SendableChooser<String> getPlacementChooser() {
		return placementChooser;
	}

	/**
	 * Returns the SendableChooser at the given index.
	 * @param index the index of the desired SendableChooser.
	 * @return the SendableChooser for the note that contains an Integer.
	 */
	public static SendableChooser<Integer> getNoteChooser(int index) {
		return noteChoosers.get(index);
	}

	/* --------------------------- SVR --------------------------- */
	// /**
	//  * Returs the selected auto path.
	//  * @return FSMState object which has the selected auto path.
	//  */
	// public static AutoPath getSelectedPath() {
	// 	return autoPathChooser.getSelected();
	// }

	// /**
	//  * Returns the selected starting position.
	//  * @return integer for the selected starting position.
	//  */
	// public static int getStartingPos() {
	// 	return startingPosChooser.getSelected();
	// }

	// /**
	//  * Returns the selected mechanism.
	//  * @return boolean for the selected mechanism.
	//  */
	// public static boolean getSelectedMech() {
	// 	return mechChooser.getSelected();
	// }
	/* --------------------------- SVR --------------------------- */

	/**
	 * Returns the selected alliance.
	 * @return boolean for the selected alliance.
	 */
	public static boolean getSelectedAlliance() {
		return allianceChooser.getSelected();
	}

	/**
	 * Returns the selected path.
	 * @return String for the selected path.
	 */
	public static String getSelectedPath() {
		return pathChooser.getSelected();
	}

	/**
	 * Returns the selected placement.
	 * @return String for the selected placement.
	 */
	public static String getSelectedPlacement() {
		return placementChooser.getSelected();
	}

	/**
	 * Returns the selected note.
	 * @param index the index of the desired note ID
	 * @return int for the selected note ID
	 */
	public static int getSelectedNote(int index) {
		return noteChoosers.get(index).getSelected();
	}
}
