package org.firstinspires.ftc.teamcode.teleop.standalone;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;


/**
 * 
 */

@TeleOp(name="Test Robot - Original", group="Linear Opmode")

public class BasicTestComplete extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    private DcMotor left_drive  = null;
    private DcMotor right_drive = null;
    private DcMotor arm_motor   = null;
    
    private Servo claw_servo    = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize hardware variables
        
        // - Motors
        left_drive  = hardwareMap.get(DcMotor.class, "Left");
        right_drive = hardwareMap.get(DcMotor.class, "Right");
        arm_motor   = hardwareMap.get(DcMotor.class, "Arm");
        
        // - Servos
        claw_servo  = hardwareMap.get(Servo.class, "Claw");
        
        // - Sensors
        ColorSensor color_sensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");
        TouchSensor arm_limit = hardwareMap.get(TouchSensor.class, "Arm_Limit");

        // Set direction
        left_drive.setDirection(DcMotor.Direction.FORWARD);
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        arm_motor.setDirection(DcMotor.Direction.REVERSE);
        

        // Wait for game to start
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Calculate Power (Drive Train)
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            
            // Calculate Power (Arm)
            double armPower = 0;
            boolean limitTripped = arm_limit.isPressed();
            
            if (!limitTripped) {
                if (gamepad1.dpad_up) {
                    armPower = .3;
                }
            } 
            
            if (gamepad1.dpad_down) {
                armPower = -.3;
            }
            
            // Set Postion (Servo) 
            double clawPosition;
            
            if (gamepad1.x) {
                claw_servo.setPosition(.5);
            } else if (gamepad1.b) {
                claw_servo.setPosition(1);
            }
            
            clawPosition = claw_servo.getPosition();
            
            // Color Sensor
            String color;
           
            // Send calculated power (Motors)
            left_drive.setPower(leftPower);
            right_drive.setPower(rightPower);
            arm_motor.setPower(armPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Train Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Arm Motor", "(%.2f)", armPower);
            telemetry.addData("Claw", "(%.2f)", clawPosition);
            telemetry.addData("Arm Tripped Limit", "(%b)", limitTripped);
            telemetry.addData("Red (%.2f)", color_sensor.red());
            telemetry.addData("Blue (%.2f)", color_sensor.blue());
            telemetry.addData("Green (%.2f)", color_sensor.green());
            telemetry.update();
            
        }
    }
}
