PWMOutputImpl ultrasonic_sensor;
    
    

public void setupUltrsonicSensor() {
        
	ultrasonic_sensor = new PWMOutputImpl(new PWMOutputController(0), 0);
    
}
    
    

public
 long getPulseWidth() {
	return (long) getPulseWidthOutputTime();
}

public long getDistanceInches(long microseconds) {
         
	return microseconds / 74 / 2;
    
}