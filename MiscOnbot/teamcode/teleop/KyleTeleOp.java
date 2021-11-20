
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class KyleTeleOp extends LinearOpMode{
    
    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    
    @Override
    public void waitForStart(){
        
    }
    public void runOpMode()
    {
        RFMotor = hardwareMap.get(DcMotor.class,"frontRight");
        RBMotor = hardwareMap.get(DcMotor.class,"backRight");
        LFMotor = hardwareMap.get(DcMotor.class,"frontLeft");
        LBMotor = hardwareMap.get(DcMotor.class,"backLeft");
        
        telemetry.addData("Status", "Running");
        telemetry.update();
        
        while(opModeIsActive()){
            double RightSpeed = 0;
            RightSpeed = gamepad1.right_stick_y;
            RFMotor.setPower(RightSpeed);
            telemetry.addData("RightSpeed: ", RightSpeed);
            
            double LeftSpeed = 0;
            LeftSpeed = gamepad1.left_stick_y;
            LFMotor.setPower(LeftSpeed);
            telemetry.addData("LeftSpeed: ", LeftSpeed);
        }
    }
}
