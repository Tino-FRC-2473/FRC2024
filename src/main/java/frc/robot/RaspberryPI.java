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
}