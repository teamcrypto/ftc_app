package org.firstinspires.ftc.robotcontroller.internal;


/**
 * Created by fwsmi on 6-10-2017.
 */

public class UserInput {
    private static final UserInput ourInstance = new UserInput();
    private FtcRobotControllerActivity act;

    public static UserInput getInstance() {
        return ourInstance;
    }

    private UserInput() {
    }

    public void setActivity(FtcRobotControllerActivity _act){
        act = _act;
    }

    public FtcRobotControllerActivity getAct(){ return act; }

    public int getNumber(){ return act.numberPicker.getValue(); }
}
