package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.Queue;

public class LimelightSubsystem extends SubsystemBase {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // Moving average window for smoothing distance
    private static final int WINDOW_SIZE = 5;
    private final Queue<Double> distanceWindow = new LinkedList<>();

    // Example camera mounting constants (adjust to your robot)
    private final double cameraHeightMeters = 0.8;
    private final double cameraAngleRadians = 0.0;

    // Example shooter RPM lookup (distance in meters -> RPM)
    private final double[] distanceBreakpoints = {2.0, 3.0, 4.0};
    private final double[] rpmBreakpoints = {3000, 3500, 4200};

    public LimelightSubsystem() {}

    /** Returns true if a target is visible */
    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    /** Returns the ID of the detected tag, -1 if none */
    public int getTagID() {
        return (int) table.getEntry("tid").getDouble(-1);
    }

    /** Returns raw distance to tag in meters using pose estimation */
    public double getRawDistanceMeters() {
        double[] pose = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        if (pose.length < 3) return 0;

        double x = pose[0]; // forward
        double y = pose[1]; // sideways
        double z = pose[2]; // vertical

        // Correct for camera tilt
        double correctedX = x * Math.cos(cameraAngleRadians) - z * Math.sin(cameraAngleRadians);
        double correctedZ = x * Math.sin(cameraAngleRadians) + z * Math.cos(cameraAngleRadians);

        return Math.sqrt(correctedX * correctedX + y * y + correctedZ * correctedZ);
    }

    /** Returns smoothed distance using moving average */
    public double getSmoothedDistance() {
        double rawDistance = getRawDistanceMeters();
        if (distanceWindow.size() >= WINDOW_SIZE) distanceWindow.poll();
        distanceWindow.add(rawDistance);

        return distanceWindow.stream().mapToDouble(d -> d).average().orElse(rawDistance);
    }

    /** Converts distance to shooter RPM via linear interpolation */
    public double getShooterRPM(double distanceMeters) {
        if (distanceMeters <= distanceBreakpoints[0]) return rpmBreakpoints[0];

        for (int i = 1; i < distanceBreakpoints.length; i++) {
            if (distanceMeters <= distanceBreakpoints[i]) {
                double ratio = (distanceMeters - distanceBreakpoints[i - 1]) /
                               (distanceBreakpoints[i] - distanceBreakpoints[i - 1]);
                return rpmBreakpoints[i - 1] + ratio * (rpmBreakpoints[i] - rpmBreakpoints[i - 1]);
            }
        }

        return rpmBreakpoints[rpmBreakpoints.length - 1];
    }

    /** Logs info for AdvantageScope / SmartDashboard */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget());
        SmartDashboard.putNumber("Limelight/TagID", getTagID());
        SmartDashboard.putNumber("Limelight/RawDistance", getRawDistanceMeters());
        SmartDashboard.putNumber("Limelight/SmoothedDistance", getSmoothedDistance());
        SmartDashboard.putNumber("Limelight/TargetRPM", getShooterRPM(getSmoothedDistance()));
    }
}