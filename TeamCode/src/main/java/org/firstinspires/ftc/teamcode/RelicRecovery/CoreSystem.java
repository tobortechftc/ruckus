package org.firstinspires.ftc.teamcode.hardware17;

import org.firstinspires.ftc.teamcode.support.YieldHandler;

/**
 * Created by 28761 on 9/4/2018.
 */

public class CoreSystem {

    private YieldHandler yieldHandler;

    public CoreSystem() {

    }

    public void set_yield_handler(YieldHandler y) {
        yieldHandler = y;
    }

    public void yield() {
        if (yieldHandler == null) {
            throw new RuntimeException("yieldHandler is null, call setYieldHandler in the beginning of teleop");
        }
        yieldHandler.on_yield();
    }

    public void yield_for(double seconds) {
        long millisec = Math.round(seconds * 1000);
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < millisec)
            yield();
    }
}