package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveConstants.AutoConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.ArrayList;

public class AutoPathChooser {
	private static SendableChooser<Boolean> allianceChooser;
	private static SendableChooser<String> pathChooser;
	private static SendableChooser<String> placementChooser;
	private static SendableChooser<Boolean> cvOptionChooser;
	private static ArrayList<SendableChooser<Integer>> noteChoosers;

	/**
	 * Constructor for creating the AutoPathChooser class. It is used in order to select the
	 * auto path and node for the robot without re-deploying.
	 */
	public AutoPathChooser() {
		allianceChooser = new SendableChooser<>();
		allianceChooser.setDefaultOption("Blue", true);
		allianceChooser.addOption("Red", false);
		SmartDashboard.putData("Alliance", allianceChooser);

		cvOptionChooser = new SendableChooser<>();
		cvOptionChooser.setDefaultOption("No", false);
		cvOptionChooser.addOption("Yes", true);
		SmartDashboard.putData("Using CV in Auto", cvOptionChooser);

		pathChooser = new SendableChooser<>();
		pathChooser.setDefaultOption("Speaker Scoring", "PROT");
		// pathChooser.addOption("Target Miscellaneous", "MISC");
		// pathChooser.addOption("Target Midfield", "MIDF");
		pathChooser.addOption("AUTO DESTROYER", "AUTO");
		pathChooser.addOption("Safety Path", "SAFE");
		SmartDashboard.putData("Path Chooser", pathChooser);

		placementChooser = new SendableChooser<>();
		placementChooser.setDefaultOption("Subwoofer Center", "SWCT");
		placementChooser.addOption("Subwoofer Source", "SWSR");
		placementChooser.addOption("Subwoofer Amp", "SWAM");
		// placementChooser.addOption("Source", "BYSR");
		// placementChooser.addOption("Amp", "BYAM");
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

	/**
	 * Returns if we're using auto in CV.
	 * @return True/False if we're using CV
	 */
	public static boolean isUsingCVAuto() {
		if (cvOptionChooser != null) {
			return cvOptionChooser.getSelected();
		} else {
			return false;
		}
	}
}
