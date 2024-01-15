package frc.robot;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.NetworkTablesConstants;

public class RaspberryPI {
	private double fps = 0;
	private NetworkTable table;

	//FPS Calculation
	private DoubleSubscriber fpsCounter;
	private DoubleArraySubscriber tagSubscriber;
	private double previousValueReceived = 0;
	private double previousTimeReceived = 0;
	private Timer timer = new Timer();

	/**Updates the FPS each iteration of the robot.*/
	public RaspberryPI() {
		timer.start();
		table = NetworkTableInstance.getDefault().getTable(NetworkTablesConstants.TABLE_NAME);
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

	//If the number 4000 is returned from any of the methods below, that output is invalid and no tag of the inputted Id has been detected
	public double getAprilTagX(int id) {
		try {
			return tagSubscriber.get()[(6 * (id - 1))];
		} catch (NullPointerException e) {
			System.out.print("CV NOT RUNNING");
			return 4000;
		}
	}

	public double getAprilTagY(int id) {
		return tagSubscriber.get()[(6 * (id - 1)) + 1];
	}

	public double getAprilTagZ(int id) {
		return tagSubscriber.get()[(6 * (id - 1)) + 2];
	}

	public double getAprilTagYaw(int id) {
		return tagSubscriber.get()[(6 * (id - 1)) + 3];
	}

	public double getAprilTagPitch(int id) {
		return tagSubscriber.get()[(6 * (id - 1)) + 4];
	}

	public double getAprilTagRoll(int id) {
		return tagSubscriber.get()[(6 * (id - 1)) + 5];
	}
}