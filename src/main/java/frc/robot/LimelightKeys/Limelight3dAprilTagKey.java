package frc.robot.LimelightKeys;

public enum Limelight3dAprilTagKey{
    GLOBAL_POSE("botpose"), 
    BLUE_ORGIN_POSE("botpose_wpiblue"), 
    RED_ORGIN_POSE("botpose_wpired"), 
    MEGATAG2_GLOBAL_POSE("botpose_orb"), 
    MEGATAG2_BLUE_ORGIN_POSE("botpose_orb_wpiblue"), 
    MEGATAG2_RED_ORGIN_POSE("botpose_orb_wpired"), 
    CAMERA_POSE_TARGET_SPACE("camerapose_targetspace"), 
    TARGET_POSE_CAMERA_SPACE("targetpose_cameraspace"), 
    TARGET_POSE_ROBOT_SPACE("targetpose_robotspace"), 
    ROBOT_POSE_TARGET_SPACE("botpose_targetspace"), 
    CAMERA_POSE_ROBOT_SPACE("camerapose_robotspace"), 
    PRIMARY_TAG_ID("tid");


    private final String key;
    Limelight3dAprilTagKey(String key) {
        this.key = key;
    }
    public String GetKey(){
        return this.key;
    }
}