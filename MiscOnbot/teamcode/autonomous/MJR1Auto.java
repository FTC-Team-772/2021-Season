package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.*;

@TeleOp
public class MJR1Auto extends OpMode{

    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    public ElapsedTime timer = new ElapsedTime();
    public void init(){
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    }
    public void loop(){
        timer.startTime();
        boolean timeReset = false;
        if (!timeReset){
            timer.reset();
            timeReset = true;
        }
        if (timer.time() < 2){
            backLeft.setPower(1);
            backRight.setPower(-1);
            frontLeft.setPower(1);
            frontRight.setPower(-1);
        } else if (timer.time() > 2){
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }
    }
}
