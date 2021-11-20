package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Arm {
    
    private Servo claw_servo        = null;
    private DcMotor arm_motor        = null;
    private TouchSensor arm_limit    = null;
    
    public boolean armLimitMet;

    public Arm(DcMotor arm_motor, Servo claw_servo, TouchSensor arm_limit) {
        this.claw_servo = claw_servo;
        this.arm_motor = arm_motor;
        this.arm_limit = arm_limit;
        
        arm_motor.setDirection(DcMotor.Direction.REVERSE);
        
        checkArmLimit();
        resetArm();
        
        //sendTelemetry(0, 0);
    }
    
    /**
     * FYI: Arm limit is checked before moving arm
     * 
     * Method will adjust arm limit boolean
     * 
     * @return Arm Limit
     */
    
    public boolean checkArmLimit() {
        armLimitMet = arm_limit.isPressed();
        return armLimitMet;
    }
    
    public void moveArm(double armPower) {
        checkArmLimit(); 
        
        if (!armLimitMet && armPower > 0) {
            moveArmNoCheck(armPower);
        }
        
        if (armPower < 0) {
            moveArmNoCheck(armPower);
        }
        
        moveArmNoCheck(0);
        
        //sendTelemetryArm(armPower);
    }
    
    /**
     * Can only be mapped to buttons that return boolean values
     */
    
    public void moveArm(boolean up, boolean down) {
        if (!armLimitMet) {
            if (up) {
                moveArmNoCheck(1);
            }
        } 
            
        if (down) {
            moveArmNoCheck(-1);
        }
        
        moveArmNoCheck(0);
    }
    
    private void moveArmNoCheck(double armPower) {
        arm_motor.setPower(armPower);
    }
    
    public void resetArm() {
        while (!armLimitMet) {
            moveArm(1);
            checkArmLimit();
            
        }
        
        moveArm(0);
    }
    
    public void moveClaw(double position) {
        claw_servo.setPosition(position);
        
        //sendTelemetryClaw(position);
    }
    
    /**
     * Can only be mapped to buttons that return boolean values
     */
    
    public void moveClaw(boolean buttonA, boolean buttonB) {
        if (buttonA) {
            moveClaw(.5);
            
        } else if (buttonB) {
            moveClaw(1);
            
        }
        
        moveClaw(0);
    }
    
    public double getClawPostion() {
        return claw_servo.getPosition();
    }
    
    /*
    
    private void sendTelemetry(double armPower, double clawPosition) {
        telemetry.addData("Arm Motor", "(%.2f)", armPower);
        telemetry.addData("Claw", "(%.2f)", clawPosition);
        telemetry.addData("Claw Limit", "(%b)", armLimitMet);
    }
    
    private void sendTelemetryArm(double armPower) {
        telemetry.addData("Arm Motor", "(%.2f)", armPower);
    }
    
    private void sendTelemetryClaw(double clawPosition) {
        telemetry.addData("Claw", "(%.2f)", clawPosition);
    }
    
    */
    
}