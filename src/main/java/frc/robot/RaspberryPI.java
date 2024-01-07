package frc.robot;
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
	private DoubleSubscriber cubeYawSubscriber;
	private DoubleSubscriber cubeDistanceSubscriber;
	private DoubleSubscriber coneYawSubscriber;
	private DoubleSubscriber coneDistanceSubscriber;
	private double previousValueReceived = 0;
	private double previousTimeReceived = 0;
	private Timer timer = new Timer();

	/**Updates the FPS each iteration of the robot.*/
	public RaspberryPI() {
		timer.start();
		table = NetworkTableInstance.getDefault().getTable(NetworkTablesConstants.TABLE_NAME);
		fpsCounter = table.getDoubleTopic(NetworkTablesConstants.FPS_COUNTER_TOPIC).subscribe(-1);
		cubeYawSubscriber = table.getDoubleTopic(
			NetworkTablesConstants.CUBE_YAW_TOPIC).subscribe(-1);
		cubeDistanceSubscriber = table.getDoubleTopic(
			NetworkTablesConstants.CUBE_DISTANCE_TOPIC).subscribe(-1);
		coneYawSubscriber = table.getDoubleTopic(
			NetworkTablesConstants.CONE_YAW_TOPIC).subscribe(-1);
		coneDistanceSubscriber = table.getDoubleTopic(
			NetworkTablesConstants.CONE_DISTANCE_TOPIC).subscribe(-1);
	}

	/**Updates the values in SmartDashboard. */
	public void update() {
		updateFPS();
		SmartDashboard.putNumber("cube yaw", getCubeYaw());
		SmartDashboard.putNumber("cube distance", getCubeDistance());
		SmartDashboard.putNumber("cone yaw", getConeYaw());
		SmartDashboard.putNumber("cone distance", getConeDistance());
	}

	/**
	 * Gets the yaw to the cube.
	 * @return returns the horizontal angle between the cube and the camera in degrees.
	 */
	public double getCubeYaw() {
		return cubeYawSubscriber.get();
	}

	/**
	 * Gets the yaw to the cone.
	 * @return returns the horizontal angle between the cone and the camera in degrees.
	 */
	public double getConeYaw() {
		return coneYawSubscriber.get();
	}

	/**
	 * Gets the distance to the cube.
	 * @return returns the distance to the cube in meters.
	 */
	public double getCubeDistance() {
		return cubeDistanceSubscriber.get();
	}

	/**
	 * Gets the distance to the cone.
	 * @return returns the distance to the cone in meters.
	 */
	public double getConeDistance() {
		return coneDistanceSubscriber.get();
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

	/**
	 * returns the FPS.
	 * @return frames per second
	 */
	public double getFPS() {
		return fps;
	}
}
