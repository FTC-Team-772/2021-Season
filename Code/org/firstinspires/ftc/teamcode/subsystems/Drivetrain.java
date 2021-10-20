package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.misc.RobotDef;

/**
* Subsystem that controls the drivetrain
* Call Methods to interface
*
*
*/

public class Drivetrain {

	// Motor Definitions
	private DcMotor backLeft = null;
    
	private DcMotor backRight = null;
    
	private DcMotor frontLeft = null;
    
	private DcMotor frontRight = null;

	// Range Sensor
	private ModernRoboticsI2cRangeSensor rightRangeSensor = null;  
	// Code Me

	public Drivetrain() {
		// Motor Hardware Maps
		backLeft = hardwareMap.get(DcMotor.class, BL_MOTOR_NAME);
        
		backRight = hardwareMap.get(DcMotor.class, BR_MOTOR_NAME);
        
		frontLeft = hardwareMap.get(DcMotor.class, FL_MOTOR_NAME);
        
		frontRight = hardwareMap.get(DcMotor.class, FR_MOTOR_NAME);


		// Reset Encoder
		backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
		backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
		frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
		frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        

		// Enable Encoders
		backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
		backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
		frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
		frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); 

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


 	/**
     
	* General purpose drive that sends values to motors
     
	*/ 
    
    

	public void drive(double leftPower, double rightPower) {
        
		 
	}
    
    

	/**
     
	* Ex. drive.driveJoystick(gamepad1.right_stick_x, gamepad1.left_stick_y);
     
	*
     
	* Can only be mapped to buttons that return position values
     
	*/
 
   
   
 	public void driveJoystick(double x, double y) {
        
		
       
        
	}

	public void rightToWall(int dist, double power, int heading) {
        
		while(rightRangeSensor.rawUltrasonic() > dist) {
            
			frontRight.setPower(-power);
            
			backRight.setPower(power);
            
			frontLeft.setPower(-power);
            
			backLeft.setPower(power);
            
			printData();
            if(angles.firstAngle>convert(2+heading)){
                backRight.setPower(power/divisor);
                backLeft.setPower(power/divisor);
            }
            else if(angles.firstAngle<convert(-2+heading)){
                frontRight.setPower(-power/divisor);
                frontLeft.setPower(-power/divisor);
            }
        }
        brake();
    }
    public void backToWall(int dist, double power, int heading){
        while(backRangeSensor.rawUltrasonic() > dist){
            frontRight.setPower(-power);
            backRight.setPower(-power);
            frontLeft.setPower(power);
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
}