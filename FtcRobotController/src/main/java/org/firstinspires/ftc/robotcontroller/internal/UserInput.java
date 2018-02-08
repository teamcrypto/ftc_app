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
    private List<Double> listDouble = new ArrayList<Double>();
    private List<Integer> min_vals = new ArrayList<Integer>();
    private List<Integer> max_vals = new ArrayList<Integer>();
    private List<String> names = new ArrayList<String>();
    private List<String> namesDouble = new ArrayList<String>();


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

    public void setMaxValue(int value){
        act.numberPicker.setMaxValue(value);
    }

    public void setMinValue(int value){
        act.numberPicker.setMinValue(value);
    }

    public void onItemSelected(int position, long id){
        act.numberPicker.setValue(list.get(position));
        act.numberPicker.setMinValue(min_vals.get(position));
        act.numberPicker.setMaxValue(max_vals.get(position));
    }

    public void onValueChange(Integer newVal){
        int position = act.dropdown.getSelectedItemPosition();
        if(position < list.size() && position != -1) {
            list.set(position, newVal);
        }
    }

    public void addVariable(Integer default_value, String name, int range_min, int range_max){
        list.add(default_value);
        min_vals.add(range_min);
        max_vals.add(range_max);
        names.add(name);
        act.listItems.add(name);
        act.finishList();
    }

    public void addVariable(Double default_value, String name, int range_min, int range_max){
        listDouble.add(default_value);
        min_vals.add(range_min);
        max_vals.add(range_max);
        namesDouble.add(name);
        act.listItems.add(name);
        act.finishList();
    }

    public double get(String name){
        int index = names.indexOf(name);
        if(index != -1){
            return ((double) list.get(index));
        }else{
            index = namesDouble.indexOf(name);
            if(index == -1){
                throw new NullPointerException();
            }else {
                return listDouble.get(index);
            }
        }
    }

    public double getDouble(String name){
        int index = names.indexOf(name);
        if(index != -1){
            return ((double) list.get(index)) / ((double) max_vals.get(index));
        }else{
            index = namesDouble.indexOf(name);
            if(index == -1){
                throw new NullPointerException();
            }else {
                return listDouble.get(index);
            }
        }
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

    /*public void setMaxValue(int value){
        act.numberPicker.setMaxValue(value);
    }

    public void setMinValue(int value){
        act.numberPicker.setMinValue(value);
    }*/


}
