package frc.robot;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RaspberryPI {
	private double fps = 0;
	private NetworkTable table;

	//FPS Calculation
	private DoubleSubscriber fpsCounter;
	private DoubleArraySubscriber tagSubscriber;
	private double previousValueReceived = 0;
	private double previousTimeReceived = 0;
	private Timer timer = new Timer();

	public static final int APRIL_TAG_CONSTANT = 6;

	/**Updates the FPS each iteration of the robot.*/
	public RaspberryPI() {
		timer.start();
		table = NetworkTableInstance.getDefault().getTable("datatable");
		fpsCounter = table.getDoubleTopic("x").subscribe(-1);
		tagSubscriber = table.getDoubleArrayTopic("april_tag_data").subscribe(null);
	}

	/**Updates the values in SmartDashboard. */
	public void update() {
		updateFPS();
	}

	/**
	 * Updates the FPS each iteration of the robot.
	 */
	public void updateFPS() {
		double currentReceivedValue = fpsCounter.get();
		if (currentReceivedValue != previousValueReceived) {
			fps = 1.0 / (timer.get() - previousTimeReceived);
			previousTimeReceived = timer.get();
		}
		previousValueReceived = currentReceivedValue;
		SmartDashboard.putNumber("FPS", fps);
	}

	//If the number 4000 is returned from any of the methods below, that output is
	//invalid and no tag of the inputted Id has been detected
	/**
	 * Gives information about april tag.
	 * @param id takes the id of tag
	 * @return the x distance to the tag in inches
	 */
	public double getAprilTagX(int id) {
		return tagSubscriber.get()[(APRIL_TAG_CONSTANT * (id - 1))];
	}
	/**
	 * Gives information about april tag.
	 * @param id takes the id of tag
	 * @return the y distance to the tag in inches
	 */
	public double getAprilTagY(int id) {
		return tagSubscriber.get()[(APRIL_TAG_CONSTANT * (id - 1)) + 1];
	}
	/**
	 * Gives information about april tag.
	 * @param id takes the id of tag
	 * @return the y distance to the tag in inches
	 */
	public double getAprilTagZ(int id) {
		return tagSubscriber.get()[(APRIL_TAG_CONSTANT * (id - 1)) + 2];
	}
	/**
	 * Gives information about april tag.
	 * @param id takes the id of tag
	 * @return the yaw to the tag
	 */
	public double getAprilTagYaw(int id) {
		return tagSubscriber.get()[(APRIL_TAG_CONSTANT * (id - 1)) + 2 + 1];
	}
	/**
	 * Gives information about april tag.
	 * @param id takes the id of tag
	 * @return the pitch to the tag
	 */
	public double getAprilTagPitch(int id) {
		return tagSubscriber.get()[(APRIL_TAG_CONSTANT * (id - 1)) + 2 + 2];
	}
	/**
	 * Gives information about april tag.
	 * @param id takes the id of tag
	 * @return the roll to the tag
	 */
	public double getAprilTagRoll(int id) {
		return tagSubscriber.get()[(APRIL_TAG_CONSTANT * (id - 1)) + 2 + 2 + 1];
	}
}
