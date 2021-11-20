package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.*;

@TeleOp
public class DrivingTeleOp extends OpMode{

    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    
    private DcMotorEx flyWheel = null;
    private DcMotor belts = null;
    private DcMotor arm = null;
    private DcMotor intake = null;
    private Servo claw = null;
    
    private static double speed = 0.6;
    private static int dir = 1;
    private static int target;
    private static double velocity, storVelocity;
    private static boolean bState, aState, yState;
    private static int degreeFactor;
    private static ElapsedTime runtime = new ElapsedTime();
    
    public void init(){

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        belts = hardwareMap.get(DcMotor.class, "belts");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");
        velocity=0;
        storVelocity=2000;
        claw.setPosition(0.1);
        degreeFactor=-arm.getCurrentPosition();
    }

    
    public void loop(){
        //Joystick and trigger driving controls (steering and throtle)
        double y = 0;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double leftT = gamepad1.left_trigger;
        double rightT = gamepad1.right_trigger;
        rightT = -rightT;
        y = leftT + rightT;
        if (leftT > 0){
            leftX = -leftX;
        }
        
        if (dir == 1){
            backLeft.setPower((y - leftX + rightX) * speed);
            backRight.setPower(-((y + leftX - rightX) * speed));
            frontLeft.setPower((y - leftX - rightX) * speed);
            frontRight.setPower(-((y + leftX + rightX) * speed));
        }
        else if (dir == -1){
            backLeft.setPower(-(y + leftX + rightX) * speed);
            backRight.setPower(((y - leftX - rightX) * speed));
            frontLeft.setPower(-(y + leftX - rightX) * speed);
            frontRight.setPower(((y - leftX + rightX) * speed));
        }
        //change speed of driving
        if (gamepad1.dpad_left){
            speed -= 0.4;
            if (speed <= .2){
                speed = 0.2;
            }
            try{
                Thread.sleep(200);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
        if (gamepad1.dpad_right){
            if (speed < .9){
                speed += 0.4;
            }
            try{
                Thread.sleep(200);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
        if (gamepad1.dpad_down){
            dir = -dir;
            try{
                Thread.sleep(200);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
        //change launcher velocity
        if(gamepad2.dpad_left && velocity>0){
            velocity=1500;
            storVelocity=1500;
            try{
                Thread.sleep(200);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
        else if(gamepad2.dpad_left && velocity==0){
            storVelocity=1500;
            try{
                Thread.sleep(200);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
        else if(gamepad2.dpad_right && velocity>0){
            velocity=2000;
            storVelocity=2000;
            try{
                Thread.sleep(200);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
        else if(gamepad2.dpad_right && velocity==0){
            storVelocity=2000;
            try{
                Thread.sleep(200);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
        else if(gamepad2.dpad_up && velocity>0){
            velocity=velocity+20;
            storVelocity=storVelocity+20;
            try{
                Thread.sleep(200);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
        else if(gamepad2.dpad_up && velocity==0){
            storVelocity=storVelocity+20;
            try{
                Thread.sleep(200);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
        else if(gamepad2.dpad_down && velocity>0){
            velocity=velocity-20;
            storVelocity=storVelocity-20;
            try{
                Thread.sleep(200);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
        else if(gamepad2.dpad_down && velocity==0){
            storVelocity=storVelocity-20;
            try{
                Thread.sleep(200);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }

        if(gamepad2.left_bumper){//turns launcher off
            velocity=0;
            try{
                Thread.sleep(200);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
        else if(gamepad2.right_bumper){//turns laucher on to past velocity number
            velocity=storVelocity;
            try{
                Thread.sleep(200);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
        
        flyWheel.setVelocity(-velocity);//sets the velocity of the flywheel to the velocity variable
        //controls the intake based on the gamepad2 left joystick
        if(gamepad2.left_stick_y>0){
            intake.setPower(-1);
            belts.setPower(-1);
        }
        else if(gamepad2.left_stick_y<0){
            intake.setPower(1);
            belts.setPower(1);
        }
        else if(gamepad2.left_stick_y==0){
            intake.setPower(0);
            belts.setPower(0);
        }
        
        arm.setPower(gamepad2.right_stick_y);
        
        if(gamepad2.right_stick_button){//autofire with right joystick pressed
            while(flyWheel.getVelocity()>-2100.00){
            try{
                Thread.sleep(100);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
        for(int i=0; i<2; i++){
                intake.setPower(1);
                belts.setPower(1);
                try{
                    Thread.sleep(500);
                }
                catch(InterruptedException e){
                    Thread.currentThread().interrupt();
                }
                intake.setPower(0);
                belts.setPower(0);
                while(flyWheel.getVelocity()>-2100.00){
                    try{
                        Thread.sleep(100);
                    }
                    catch(InterruptedException e){
                        Thread.currentThread().interrupt();
                    }
                }
            }
            intake.setPower(1);
            belts.setPower(1);
            try{
                Thread.sleep(500);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
            intake.setPower(0);
            belts.setPower(0);
        }
        
        
        //controls the claw opening and closing based on the triggers of the gamepad2
        if(gamepad2.left_trigger>0){//open claw
            claw.setPosition(0.5);
        }
        else if(gamepad2.right_trigger>0){//closed claw
            claw.setPosition(0.1);
        }
        //controls the position of the wooble goal arm based on lettered buttons of gamepad2
        if(gamepad2.a){//goes to position a
            target=0-degreeFactor;
            aState=true;
            bState=false;
            yState=false;
        }
        //need to swap b and y
        else if(gamepad2.y){//goes to position y
            target=245-degreeFactor;
            aState=false;
            yState=true;
            bState=false;
        }
        else if(gamepad2.b){//goes to position b
            target=575-degreeFactor;
            aState=false;
            yState=false;
            bState=true;
        }
        else if(gamepad2.x){//turns motor off
            aState=false;
            bState=false;
            yState=false;
            degreeFactor=-arm.getCurrentPosition();
        }
        else if(Math.abs(arm.getCurrentPosition()-target)<=10){
            aState=false;
            bState=false;
            yState=false;
        }
        //if button a, b, or y has been pressed, the motor will constantly try to be at the target position
        if(yState || bState || aState){
            if(arm.getCurrentPosition()>target-10){
                arm.setTargetPosition(target);
                arm.setPower(-0.5);
            }
            else if(arm.getCurrentPosition()<target+10){
                arm.setTargetPosition(target);
                arm.setPower(0.5);
            }
        }
        else if(aState==false && bState==false && yState==false){//if x button is pressed the motor is turned off
            arm.setPower(0);
        }
        
        // telemetry.addData("y", y);
        // telemetry.addData("leftX", leftX);
        // telemetry.addData("rightX", rightX);
        // telemetry.addData("leftT", leftT);
        // telemetry.addData("rightT", rightT);
        telemetry.addData("Direction", dir);
        telemetry.addData("Speed", speed);
        telemetry.addData("DegreeFactor:", degreeFactor);
        telemetry.addData("Arm Position", arm.getCurrentPosition());
        telemetry.addData("Velocity", velocity);
        telemetry.addData("storVelocity", storVelocity);
        telemetry.addData("Flywheel Velocity", -flyWheel.getVelocity()); //make it a var
        telemetry.addData("right joy", gamepad2.right_stick_y);
    }
}