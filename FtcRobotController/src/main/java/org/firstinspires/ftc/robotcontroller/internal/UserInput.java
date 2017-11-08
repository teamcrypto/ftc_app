package org.firstinspires.ftc.robotcontroller.internal;


import android.widget.NumberPicker;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by fwsmi on 6-10-2017.
 */

public class UserInput {
    private static final UserInput ourInstance = new UserInput();
    private FtcRobotControllerActivity act;

    private List<Integer> list = new ArrayList<Integer>();
    private List<Integer> range_min = new ArrayList<Integer>();


    public static UserInput getInstance() {
        return ourInstance;
    }

    private UserInput() {
    }

    public void setActivity(FtcRobotControllerActivity _act){
        act = _act;
    }

    public FtcRobotControllerActivity getAct(){ return act; }

    //public int getNumber(){ return act.numberPicker.getValue(); }

    public void setup(){
        act.listItems.clear();
        list.clear();
    }


    public void onItemSelected(int position, long id){
        act.numberPicker.setValue(list.get(position));
    }

    public void onValueChange(Integer newVal){
        int position = act.dropdown.getSelectedItemPosition();
        if(position < list.size() && position != -1) {
            list.set(position, newVal);
        }
    }

    public void addVariable(Integer variable, String name, int range_min, int range_max){
        list.add(variable);
        act.listItems.add(name);
        act.finishList();
        //valueRef = variable;
        variable = 3;
        act.showUI();
    }

    public boolean isButtonReleased() {
        //static boolean buttonState = act.but
        //boolean lastButtonState;
        return false;
    }
    int position = 0;
    public Integer getValue(){
        int value = list.get(position);
        position = (position + 1) % list.size();
        return value;
    }

    public void hideUI(){
        act.hideUI();
    }

    public void showUI(){
        act.showUI();
    }

    public void setMaxValue(int value){
        act.numberPicker.setMaxValue(value);
    }

    public void setMinValue(int value){
        act.numberPicker.setMinValue(value);
    }


}
