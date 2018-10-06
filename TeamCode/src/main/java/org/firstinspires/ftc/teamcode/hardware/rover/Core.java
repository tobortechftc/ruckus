package org.firstinspires.ftc.teamcode.hardware.rover;

import org.firstinspires.ftc.teamcode.support.YieldHandler;

/**
 * Created by 28761 on 9/4/2018.
 */

public class Core {

    private YieldHandler yieldHandler;

    Core() {

    }

    public void set_yield_handler(YieldHandler y) {
        yieldHandler = y;
    }

    public void yield(){
        yieldHandler.on_yield();
    }

    public void yield_for(double seconds) {
        long millisec = Math.round(seconds * 1000);
        try {
            Thread.sleep(millisec);
        } catch (InterruptedException e) {
            return;
        }
    }

}
