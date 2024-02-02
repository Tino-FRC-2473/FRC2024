package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.NetworkTablesConstants;
import frc.robot.SwerveConstants.VisionConstants;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class RaspberryPI {
	private double fps = 0;
	private NetworkTable table;
	private CvSource outputStream;

	private DoubleSubscriber fpsCounter;
	private DoubleArraySubscriber tagSubscriber;
	private DoubleArraySubscriber outputFrameSub;
	private double previousValueReceived = 0;
	private double previousTimeReceived = 0;
	private Timer timer = new Timer();
	public static final int VALUES_PER_TAG = 6;

	/**Updates the FPS each iteration of the robot.*/
	public RaspberryPI() {
		outputStream = CameraServer.putVideo("Scoring cam", 320, 240);
		timer.start();
		table = NetworkTableInstance.getDefault().getTable(NetworkTablesConstants.TABLE_NAME);
		fpsCounter = table.getDoubleTopic("x").subscribe(-1);
		tagSubscriber = table.getDoubleArrayTopic("april_tag_data").subscribe(null);
		outputFrameSub = table.getDoubleArrayTopic("output_stream").subscribe(null);
	}

	/**Updates the values in SmartDashboard. */
	public void update() {
		updateFPS();
		updateStream();
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
	 * Updates the stream each iteration of the robot.
	 */
	public void updateStream() {
		try {
			int rows = 240;
			int cols = 320;

			Mat reshapedMat = new Mat(rows, cols, CvType.CV_8UC3);

			reshapedMat.put(0, 0, outputFrameSub.get());
			outputStream.putFrame(reshapedMat);
		} catch (Exception e) {
			System.out.println(e.getMessage());
		}
	}
	/**
	 * @param id id of the april tag we are fetching data on
	 * @return X value from the tag to camera in meters
	 * This value is used in tag-relative swerve movements
	 */
	public double getAprilTagX(int id) {
		try {
			return tagSubscriber.get()[(VALUES_PER_TAG * (id - 1))];
		} catch (NullPointerException e) {
			return VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT;
		}
	}

	/**
	 * @param id id of the april tag we are fetching data on
	 * @return Y value from the tag to camera in meters
	 * This value is used in tag-relative swerve movements
	 */
	public double getAprilTagY(int id) {
		try {
			return tagSubscriber.get()[(VALUES_PER_TAG * (id - 1)) + 1];
		} catch (NullPointerException e) {
			return VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT;
		}
	}

	/**
	 * @param id id of the april tag we are fetching data on
	 * @return Z value from the tag to camera in meters
	 * This value is used in tag-relative swerve movements
	 */
	public double getAprilTagZ(int id) {
		try {
			return tagSubscriber.get()[(VALUES_PER_TAG * (id - 1)) + 2];
		} catch (NullPointerException e) {
			return VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT;
		}
	}

	/**
	 * @param id id of the april tag we are fetching data on
	 * @return X value from the camera to tag in meters
	 * This value is proportional to yaw and is used in robot-relative swerve movements
	 */
	public double getAprilTagXInv(int id) {
		try {
			return tagSubscriber.get()[(VALUES_PER_TAG * (id - 1)) + 2 + 1];
		} catch (NullPointerException e) {
			return VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT;
		}
	}

	/**
	 * @param id id of the april tag we are fetching data on
	 * @return Y value from the camera to tag in meters
	 * This value is proportional to pitch and is used in robot-relative swerve movements
	 */
	public double getAprilTagYInv(int id) {
		try {
			return tagSubscriber.get()[(VALUES_PER_TAG * (id - 1)) + 2 + 2];
		} catch (NullPointerException e) {
			return VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT;
		}
	}

	/**
	 * @param id id of the april tag we are fetching data on
	 * @return Z value from the camera to tag in meters
	 * This value is used in robot-relative swerve movements
	 */
	public double getAprilTagZInv(int id) {
		try {
			return tagSubscriber.get()[(VALUES_PER_TAG * (id - 1)) + 2 + 2 + 1];
		} catch (NullPointerException e) {
			return VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT;
		}
	}
}
