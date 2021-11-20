package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


public class DriveTrain {

    // Motors
    private DcMotor left_drive  = null;
    private DcMotor right_drive = null;
    
    public DriveTrain(DcMotor left_drive, DcMotor right_drive) {
        // Set-up motors
        this.left_drive = left_drive;
        this.right_drive = right_drive;
        
        // Set direction
        left_drive.setDirection(DcMotor.Direction.FORWARD);
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        
    }
    
    /**
     * General purpose drive that sends values to motors
     */ 
    
    public void drive(double leftPower, double rightPower) {
        // Send calculated power (Motors)
        left_drive.setPower(leftPower);
        right_drive.setPower(rightPower);
        
        //sendTelemetry(leftPower, rightPower);
    }
    
    /**
     * Ex. drive.driveJoystick(gamepad1.right_stick_x, gamepad1.left_stick_y);
     *
     * Can only be mapped to buttons that return position values
     */
    
    public void driveJoystick(double x, double y) {
        y = -y;
        
        double leftPower;
        double rightPower;
        
        // Calculate Power (Drive Train)
        leftPower = Range.clip(y + x, -1.0, 1.0) ;
        rightPower = Range.clip(y - x, -1.0, 1.0) ;
        
        drive(leftPower, rightPower);
        
        /* 
        // Calculate Power (Drive Train)
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower     = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        */
    }
    
    /*
    
    private void sendTelemetry(double leftPower, double rightPower) {
        // Send telemetry
        telemetry.addData("Drive Train Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        
    }
    
    */

}