package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ColorDetection {
    
    public double red;
    public double green;
    public double blue;
    
    private ColorSensor color_sensor;
    
    public ColorDetection(ColorSensor color_sensor) {
        this.color_sensor = color_sensor;
        
        red = 0;
        blue = 0;
        green = 0;
    }
    
    public void updateColor() {
        red = color_sensor.red();
        blue = color_sensor.blue();
        green = color_sensor.green();
    }
    
    /**
     * Super basic
     */
    
    public boolean checkForRed() {
        updateColor();
        
        if (red > (green-(green*.1)) && red > (blue-(blue*.1))) {
            return true;
        } else {
            return false;
        }
    }
    
    /**
     * Super basic
     */
    
    public boolean checkForBlue() {
        updateColor();
        
        if (blue > (green-(green*.1)) && blue > (red-(red*.1))) {
            return true;
        } else {
            return false;
        }
    }
    
    public String getColor() {
        return null;
    }
    
    /*
    
    public void recordIndividualValues() {
        telemetry.addData("Red (%.2f)", color_sensor.red());
        telemetry.addData("Blue (%.2f)", color_sensor.blue());
        telemetry.addData("Green (%.2f)", color_sensor.green());
    }
    
    */
}