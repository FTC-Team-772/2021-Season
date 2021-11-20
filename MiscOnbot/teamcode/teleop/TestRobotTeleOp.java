package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.*;

@TeleOp(name="Test Robot", group="Linear Opmode")

public class TestRobotTeleOp extends LinearOpMode {
    
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        // Post update
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Set up devices
        TouchSensor arm_limit     = hardwareMap.get(TouchSensor.class, "Arm_Limit");
        DcMotor arm_motor         = hardwareMap.get(DcMotor.class, "Arm");
        Servo claw_servo         = hardwareMap.get(Servo.class, "Claw");
        
        DcMotor left_drive        = hardwareMap.get(DcMotor.class, "Left");
        DcMotor right_drive      = hardwareMap.get(DcMotor.class, "Right");
        
        ColorSensor color_sensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");
        
        // Set up classes
        DriveTrain drive = new DriveTrain(left_drive, right_drive);
        Arm arm = new Arm(arm_motor, claw_servo, arm_limit);
        
        ColorDetection color = new ColorDetection(color_sensor);
        
        // Wait for game to start
        waitForStart();
        runtime.reset();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // Run
            drive.driveJoystick(gamepad1.right_stick_x, gamepad1.left_stick_y);
            arm.moveArm(gamepad1.dpad_up, gamepad1.dpad_down);
            arm.moveClaw(gamepad1.x, gamepad1.b);
            
            // Telemetry
            //color.recordIndividualValues();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
        }
    }
}
