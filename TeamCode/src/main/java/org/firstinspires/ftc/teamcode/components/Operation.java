package org.firstinspires.ftc.teamcode.components;

/**
 * Will be returned from operations (commands) that may take some time to complete.
 */
public interface Operation {

    /**
     * Indicates whether the operation (command) requested is finished
     */
    boolean isFinished();
}
