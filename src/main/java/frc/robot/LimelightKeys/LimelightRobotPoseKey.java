package frc.robot.LimelightKeys;

//Only use for the following: botpose, botpose_wpiblue, botpose_wpired, 
//botpose_orb, botpose_orb_wpiblue, botpose_orb_wpired
public enum LimelightRobotPoseKey{
    POSE(0),
    POSE_Y(1),
    POSE_Z(2), 
    POSE_ROTATION_X(3),
    POSE_ROTATION_Y(4), 
    POSE_ROTATION_Z(5),
    TOTAL_LATENCY(6),
    TAG_COUNT(7), 
    TAG_SPAN(8), 
    AVR_TAG_DISTANCE_FROM_CAMERA(9), 
    AVR_TAG_AREA(10);
    
    private final int key;
    LimelightRobotPoseKey(int key) {
        this.key = key;
    }

    public int GetKey(){
        return this.key;
    }
}