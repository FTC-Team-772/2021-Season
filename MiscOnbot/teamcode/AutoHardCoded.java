package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.Locale;
import java.util.Locale;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
@TeleOp
public class AutoHardCoded extends LinearOpMode
{
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    
    private DcMotorEx flyWheel = null;
    private DcMotor belts = null;
    private DcMotor arm = null;
    private DcMotor intake = null;
    private Servo claw = null;
    private ColorSensor topSensor;
    private ColorSensor bottomSensor;
    private int numberRings=-1;
    private int north=0,east=-90;
    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    private int target,divisor=3;
    private double velocity, storVelocity;
    private int degreeFactor;
    private double topDistance,bottomDistance;
    
    
    public void runOpMode()
    {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        belts = hardwareMap.get(DcMotor.class, "belts");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //A bunch of imu stuff that I don't understand
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        ElapsedTime runtime = new ElapsedTime();//.reset() then milliseconds() gives time from last reset()
        
        //Color sensor code
        topSensor = hardwareMap.get(ColorSensor.class, "topSensor");
        bottomSensor = hardwareMap.get(ColorSensor.class, "bottomSensor");
        claw.setPosition(0.1);
        degreeFactor=-arm.getCurrentPosition();
        forward(1.9,1.0,north);//forward 9 rotations at 0.2 power at a heading of 0 degrees
        int total=0,entries=0;
        double distance=(1.5*360)+frontRight.getCurrentPosition();//one rotation goes 11.87in
        while(frontRight.getCurrentPosition()<distance){//right side positive lefts side negative
            frontRight.setTargetPosition((int)distance);
            frontRight.setPower(0.2);
            backRight.setTargetPosition((int)distance);
            backRight.setPower(0.2);
            frontLeft.setTargetPosition((int)distance);
            frontLeft.setPower(-0.2);
            backLeft.setTargetPosition((int)distance);
            backLeft.setPower(-0.2);
            printData();
            if(angles.firstAngle>convert(2+north)){
                frontRight.setPower(0);
                backRight.setPower(0);
            }
            else if(angles.firstAngle<convert(-2+north)){
                frontLeft.setPower(0);
                backLeft.setPower(0);
            }
            topDistance = ((DistanceSensor) topSensor).getDistance(DistanceUnit.CM);
            bottomDistance = ((DistanceSensor) bottomSensor).getDistance(DistanceUnit.CM);
            printData();
            if(bottomDistance<2.0 && numberRings==-1){
                topDistance = ((DistanceSensor) topSensor).getDistance(DistanceUnit.CM);
                bottomDistance = ((DistanceSensor) bottomSensor).getDistance(DistanceUnit.CM);
                if(topDistance>3 && bottomDistance<3){
                    numberRings=1;
                }
                else if(topDistance<3 && bottomDistance<3){
                    numberRings=4;
                }
            }
        }
        if(numberRings==-1){
            numberRings=0;
        }
        brake();
        if(numberRings==0){
            strafeRight(3,1.0,north);
            forward(1.20,1.0,north);
            armPosition(525-degreeFactor, 0.5);
            clawPosition(0.5);
            strafeRight(0.6,1.0,north);
            backward(6.2,1.0,north);
            armPosition(245-degreeFactor, 0.5); //brings up the arm
            strafeLeft(3.25,0.5,north);//strafes past 2nd wooble goal
            armPosition(525-degreeFactor, 0.5);//lowers arm
            clawPosition(0.9);//opens claw to max
            backward(0.2,1.0,north);//backs up to square itself
            strafeRight(1,0.5,north);//strafes right to wooble goal
            clawPosition(0.1);//picks up goal
            armPosition(245-degreeFactor, 0.5);
            strafeRight(0.6,1.0,north);//goes to wall
            forward(5.0,1.0,north);
            armPosition(525-degreeFactor, 0.5);
            clawPosition(0.5);
            strafeLeft(1.5,1.0,north);
            forward(0.6,1.0,north);
            //armPosition(100-degreeFactor, 0.5);
        }
        else if(numberRings==1){
            strafeLeft(1,1.0,north);
            forward(4.5,1.0,north);
            strafeRight(1.75,1.0,north);
            armPosition(525-degreeFactor, 0.5);
            clawPosition(0.5);
            armPosition(245-degreeFactor, 0.5);
            strafeRight(4.2,1.0,north);
            backward(9.5,1.0,north);
            strafeLeft(3.25,0.5,north);//strafes past 2nd wooble goal
            armPosition(525-degreeFactor, 0.5);//lowers arm
            clawPosition(0.9);//opens claw to max
            backward(0.2,1.0,north);//backs up to square itself
            strafeRight(1,0.5,north);//strafes right to wooble goal
            clawPosition(0.1);//picks up goal
            armPosition(245-degreeFactor, 0.5);
            strafeLeft(3,1.0,north);
            forward(8,1.0,north);
            strafeRight(2,1.0,north);
            armPosition(525-degreeFactor, 0.5);
            clawPosition(0.5);
            //armPosition(100-degreeFactor, 0.5);
        }
        else{
            backward(0.1,1.0,north);//backs up from rings
            strafeRight(3,1.0,north);//goes right
            forward(6.5,1.0,north);//goes forward to target area
            armPosition(525-degreeFactor, 0.5);//drops goal
            clawPosition(0.5);
            armPosition(245-degreeFactor, 1.0);//raises arm
            strafeRight(0.7,0.5,north);//goes to wall
            backward(13.5,1.0,north);//backs up to back right corner
            strafeLeft(3.25,0.5,north);//strafes past 2nd wooble goal
            armPosition(525-degreeFactor, 0.5);//lowers arm
            clawPosition(0.9);//opens claw to max
            backward(0.2,1.0,north);//backs up to square itself
            strafeRight(1,0.5,north);//strafes right to wooble goal
            clawPosition(0.1);//picks up goal
            armPosition(245-degreeFactor, 0.5);
            strafeRight(2,0.5,north);//goes to wall
            forward(11,1.0,north);//goes forward wall riding to target area
            strafeLeft(0.9,1.0,north);//strafe left to avoid other goal in area
            armPosition(525-degreeFactor, 0.5);//drops goal
            clawPosition(0.5);
            backward(2.5,1.0,north);//backs up to line
            strafeLeft(2,1.0,north);//strafe left to be on line
            //armPosition(100-degreeFactor, 1.0);//reset arm
        }
        try{
            Thread.sleep(1000);
        }
        catch(InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
    
    public double distance(double average, double rings){
        double answer=Math.pow(average-rings,2);
        return answer;
    }
    
    public void clawPosition(double target){
        claw.setPosition(target);
        try{
                Thread.sleep(1000);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
    }
    
    public void armPosition(double target, double power){
        while(Math.abs(arm.getCurrentPosition()-target)>10){
            if(arm.getCurrentPosition()>target-10){
                arm.setTargetPosition((int)target);
                arm.setPower(-power);
            }
            else if(arm.getCurrentPosition()<target+10){
                arm.setTargetPosition((int)target);
                arm.setPower(power);
            }
        }
        brake();
    }
    
    public void forward(double rotations, double power, int heading)
    {
        double target=(rotations*360)+frontRight.getCurrentPosition();//one rotation goes 11.87in
        while(frontRight.getCurrentPosition()<target){//right side positive lefts side negative
            frontRight.setTargetPosition((int)target);
            frontRight.setPower(power);
            backRight.setTargetPosition((int)target);
            backRight.setPower(power);
            frontLeft.setTargetPosition((int)target);
            frontLeft.setPower(-power);
            backLeft.setTargetPosition((int)target);
            backLeft.setPower(-power);
            printData();
            if(angles.firstAngle>convert(2+heading)){
                frontRight.setPower(power/divisor);
                backRight.setPower(power/divisor);
            }
            else if(angles.firstAngle<convert(-2+heading)){
                frontLeft.setPower(-power/divisor);
                backLeft.setPower(-power/divisor);
            }
        }
        brake();
    }
    
    public void backward(double rotations, double power, int heading)
    {
        double target=(-rotations*360)+frontRight.getCurrentPosition();//one rotation goes 11.87in
        while(frontRight.getCurrentPosition()>target){//right side positive lefts side negative
            frontRight.setTargetPosition((int)target);
            frontRight.setPower(-power);
            backRight.setTargetPosition((int)target);
            backRight.setPower(-power);
            frontLeft.setTargetPosition((int)target);
            frontLeft.setPower(power);
            backLeft.setTargetPosition((int)target);
            backLeft.setPower(power);
            printData();
            if(angles.firstAngle>convert(2+heading)){
                frontLeft.setPower(power/divisor);
                backLeft.setPower(power/divisor);
            }
            else if(angles.firstAngle<convert(-2+heading)){
                frontRight.setPower(-power/divisor);
                backRight.setPower(-power/divisor);
            }
        }
        brake();
    }
    
    public void strafeRight(double rotations, double power, int heading){
        double target=(rotations*360)+backRight.getCurrentPosition();
        while((double)backRight.getCurrentPosition()<target){
            frontRight.setTargetPosition((int)target);
            frontRight.setPower(-power);
            backRight.setTargetPosition((int)target);
            backRight.setPower(power);
            frontLeft.setTargetPosition((int)target);
            frontLeft.setPower(-power);
            backLeft.setTargetPosition((int)target);
            backLeft.setPower(power);
            printData();
            if(angles.firstAngle>convert(2+heading)){
                backRight.setPower(power/divisor);
                backLeft.setPower(power/divisor);
            }
            else if(angles.firstAngle<convert(-2+heading)){
                frontRight.setPower(-power/divisor);
                frontLeft.setPower(-power/divisor);
                
                //strafe left = right side spin outwards; left side spins inwards
            }
        }
        brake();
    }
    
    public void strafeLeft(double rotations, double power, int heading){
        double target=(-rotations*360)+backRight.getCurrentPosition();
        while((double)backRight.getCurrentPosition()>target){
            frontRight.setTargetPosition((int)target);
            frontRight.setPower(power);
            backRight.setTargetPosition((int)target);
            backRight.setPower(-power);
            frontLeft.setTargetPosition((int)target);
            frontLeft.setPower(power);
            backLeft.setTargetPosition((int)target);
            backLeft.setPower(-power);
            printData();
            if(angles.firstAngle>convert(2+heading)){
                frontRight.setPower(power/divisor);
                frontLeft.setPower(power/divisor);
            }
            else if(angles.firstAngle<convert(-2+heading)){
                backRight.setPower(-power/divisor);
                backLeft.setPower(-power/divisor);
            }
        }
        brake();
    }
    
    public void turnRight(double power, int heading){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while(angles.firstAngle>heading){
             frontLeft.setPower(-0.2);
             backLeft.setPower(-0.2);
             frontRight.setPower(-0.2);
             backRight.setPower(-0.2);
             printData();
        }
        brake();
    }
    
    public void turnLeft(double power, int heading){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while(angles.firstAngle<heading){
             frontLeft.setPower(0.2);
             backLeft.setPower(0.2);
             frontRight.setPower(0.2);
             backRight.setPower(0.2);
             printData();
        }
        brake();
    }
    
    public void brake(){
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        arm.setPower(0);
        try{
            Thread.sleep(200);
        }
        catch(InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
    
    public void shoot(){
        flyWheel.setVelocity(-2200);
        while(flyWheel.getVelocity()>-2100.00){
            try{
                Thread.sleep(100);
            }
            catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
            printData();
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
                    printData();
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
            flyWheel.setPower(0.0);
    }
    
    public void printData(){
        telemetry.addData("backLeft Position",backLeft.getCurrentPosition());
        telemetry.addData("backRight Position",backRight.getCurrentPosition());
        telemetry.addData("frontLeft Position",frontLeft.getCurrentPosition());
        telemetry.addData("frontRight Position",frontRight.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading",angles.firstAngle);
        telemetry.addData("Flywheel",flyWheel.getVelocity());
        topDistance = ((DistanceSensor) topSensor).getDistance(DistanceUnit.CM);
        bottomDistance = ((DistanceSensor) bottomSensor).getDistance(DistanceUnit.CM);
        telemetry.addData("Top Distance", topDistance);
        telemetry.addData("Bottom Distance", bottomDistance);
        telemetry.addData("Number Rings", numberRings);
        telemetry.update();
    }
    
    public int convert(int angle){
        if(angle>180){
            return angle-360;
        }
        else if(angle<-180){
            return angle+360;   
        }
        else{
            return angle;
        }
    }
}