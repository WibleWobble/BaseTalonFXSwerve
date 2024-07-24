package frc.robot.LimelightKeys;

public enum LimelightTargetingKey{
    IS_VALID_TARGET("tv"), 
    HORIZONTAL_OFFSET("tx"), 
    VERTICAL_OFFSET("ty"), 
    HORIZONTAL_OFFSET_FROM_PRINCIPAL_PIXEL("txnc"), 
    VERTICAL_OFFSET_FROM_PRINCIPAL_PIXEL("tync"), 
    TARGET_AREA("ta"), 
    PIPELINE_LATENCY("tl"), 
    CAPTURE_LATENCY("cl"), 
    BOUNDING_BOX_SHORTEST_SIDE("tshort"), 
    BOUNDING_BOX_LONGEST_SIDE("tlong"), 
    ROUGH_BOUNDING_BOX_HORIZONTAL("thor"), 
    ROUGH_BOUNDING_BOX_VERTICAL("tvert"), 
    GET_PIPELINE("getpipe"), 
    GET_JSON("json"), 
    NEURAL_NETWORK_RESULT_CLASS("tclass"), 
    AVR_HSV_AROUND_CROSSHAIR("tc"), 
    HEARTBEAT("hb"), 
    SYSTEM_METRICS("hw"), 
    PRIORITY_TAG_ID("priorityid");//Is a setter and only affects

    

    private final String key;
    LimelightTargetingKey(String key) {
        this.key = key;
    }

    public String GetKey(){
        return this.key;
    }
}