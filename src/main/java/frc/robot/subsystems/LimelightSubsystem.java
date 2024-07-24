package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightKeys.Limelight3dAprilTagKey;
import frc.robot.LimelightKeys.LimelightRobotPoseKey;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase{
    public double latency;  //I don't know which way is best, so I have included both methods...
    public Pose3d botPose3d;//Regarding using a public variable that gets updated via the 
                            //"periodic" function or using a function
    
    private NetworkTable table;
    private String m_limelightID;
    private String orgin;

    public LimelightSubsystem(String limeLightID, boolean isBlue){
        m_limelightID = limeLightID;
        table = NetworkTableInstance.getDefault().getTable(m_limelightID);
        if (isBlue){
            orgin = Limelight3dAprilTagKey.BLUE_ORGIN_POSE.GetKey();
        }else{
            orgin = Limelight3dAprilTagKey.RED_ORGIN_POSE.GetKey();
        }
    }

    @Override
    public void periodic() {
        latency = table.getEntry(orgin).getDoubleArray(new double[6])[LimelightRobotPoseKey.TOTAL_LATENCY.GetKey()];
        botPose3d = toPose3d(table.getEntry(orgin).getDoubleArray(new double[6]));
    }

    public double getLatency(Limelight3dAprilTagKey key){return table.getEntry(key.GetKey()).getDoubleArray(new double[6])[LimelightRobotPoseKey.TOTAL_LATENCY.GetKey()];}
    public Pose3d getPose3d(Limelight3dAprilTagKey key){return toPose3d(table.getEntry(key.GetKey()).getDoubleArray(new double[6]));}
    public Pose2d getPose2d(Limelight3dAprilTagKey key){return new Pose2d(getPose3d(key).getX(), getPose3d(key).getY(), getPose3d(key).getRotation().toRotation2d());} //This may be inefficient and will likly be changed in the future but it works for now...
    

    private static Pose3d toPose3d(double[] ntValues) {
        if (ntValues.length >= 6) {
            return new Pose3d(new Translation3d(ntValues[0], ntValues[1], ntValues[2]), new Rotation3d(Math.toRadians(ntValues[3]), Math.toRadians(ntValues[4]), Math.toRadians(ntValues[5])));
        } else {
            return null;
        }
    }
}
