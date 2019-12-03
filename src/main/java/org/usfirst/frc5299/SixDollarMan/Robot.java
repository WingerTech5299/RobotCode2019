package org.usfirst.frc5299.SixDollarMan;

//import com.ctre.CANTalon;

import edu.wpi.cscore.VideoSource;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

    public double RightMotor;
    public double LeftMotor;
    public static RobotDrive DriveTrain;
    public static RobotDrive IntakeDrive;
    public static RobotDrive LebronJames;
    public static RobotDrive NotArm;
	public Joystick LeftJoystick;
	public Joystick RightJoystick;
	public double Red;
	public double Green;
	public double Blue;
	public double RedOut;
	public double GreenOut;
    public double BlueOut;
    public double linearActuatorOutput;
	int autoLoopCounter;
	Victor leftMotor, rightMotor;
	Jaguar Shooter;
	public CameraServer server;
    public VideoSource cam0;
    //public VideoSource cam1;
    public DigitalInput forwardLimitSwitch, reverseLimitSwitch;
    public DigitalInput upLimitSwitch, downLimitSwitch;
    public boolean ArmDisplay;
    public int ArmTracer;
    public boolean WristDisplay;
    public int WristTracer;
    public boolean DarthVatorDisplay;
    public int DarthVatorTracer;
    public boolean WheelieDisplay;
    public int WheelieTracer;
    public double IntakeTracer;

    public DoubleSolenoid nintendoDS;

    public int IntakeTracerTracer;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        forwardLimitSwitch = new DigitalInput(0);
        reverseLimitSwitch = new DigitalInput(1);
        upLimitSwitch = new DigitalInput(2);
        downLimitSwitch = new DigitalInput(3);
        DriveTrain = new RobotDrive(7,8);
        IntakeDrive = new RobotDrive(2,9);
        LebronJames = new RobotDrive(1,6);
        NotArm = new RobotDrive(5,0);
    	LeftJoystick = new Joystick(0);
        RightJoystick = new Joystick(1);

        
        CameraServer.getInstance().startAutomaticCapture();
        //CameraServer.getInstance().startAutomaticCapture(1);
        nintendoDS = new DoubleSolenoid(0,1);
        Compressor c = new Compressor(0);

    	c.setClosedLoopControl(true);
    	/**AnalogOutput RedOut = new AnalogOutput(0);
    	AnalogOutput GreenOut = new AnalogOutput(1);
    	AnalogOutput BlueOut = new AnalogOutput(2);**/
    	
    	
    	System.out.println("In robotInit");
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	//autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	teleopPeriodic();
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        double LeftMotorMedium = (-LeftJoystick.getRawAxis(1) + LeftJoystick.getRawAxis(2));
        if(LeftMotorMedium>=1){
            LeftMotor=1;
        }
        else if(LeftMotorMedium<=-1){
            LeftMotor=-1;
        }
        else{
            LeftMotor = (-LeftJoystick.getRawAxis(1) + LeftJoystick.getRawAxis(2));
        }

        double RightMotorMedium = (-LeftJoystick.getRawAxis(1) - LeftJoystick.getRawAxis(2));
        if(RightMotorMedium>=1){
            RightMotor=1;
        }
        else if(RightMotorMedium<=-1){
            RightMotor=-1;
        }
        else{
            RightMotor = (-LeftJoystick.getRawAxis(1) - LeftJoystick.getRawAxis(2));
        }

        if((RightJoystick.getRawButton(2)==true)) //changed from (2) - Fred-Tim Fast-John
        {
        	//wonderValue = 1.0;
        	
        	nintendoDS.set(DoubleSolenoid.Value.kForward);
        	
        }
        else if((RightJoystick.getRawButton(2)==false)) //Also changed from (2) - FTFJ
        {
        	//wonderValue = 0;
        	if((RightJoystick.getRawButton(3)==true))
        	{
        	nintendoDS.set(DoubleSolenoid.Value.kReverse);
        }
        	else{
        		//nintendoDS.set(DoubleSolenoid.Value.kOff);
        	}
        }
 
        SmartDashboard.putNumber("RM", RightMotor);
        SmartDashboard.putNumber("LM", LeftMotor);
        SmartDashboard.putNumber("LMM", LeftMotorMedium);
        SmartDashboard.putNumber("RMM", RightMotorMedium);
        SmartDashboard.putBoolean("ARM", ArmDisplay);
        SmartDashboard.putBoolean("WRIST", WristDisplay);
        SmartDashboard.putBoolean("VATOR", DarthVatorDisplay);
        SmartDashboard.putBoolean("WHEELIE", WheelieDisplay);

        //LinearActuatorOutput has been repurposed for the wheelie bar
        double linearActuatorOutput = RightJoystick.getRawAxis(1); //Moves the joystick based on Y value
        if (!forwardLimitSwitch.get()){ // If the forward limit switch is pressed, we want to keep the values between -1 and 0
            if(RightJoystick.getRawAxis(1) < 0){
                linearActuatorOutput = 0;
            }
            else{
                linearActuatorOutput = 1;
            }
        }
        else if(!reverseLimitSwitch.get()){ // If the reversed limit switch is pressed, we want to keep the values between 0 and 1
            if(RightJoystick.getRawAxis(1) >= 0){
                linearActuatorOutput = 0;
            }
            else{
                linearActuatorOutput = 1;
            }
        }
        else{
            linearActuatorOutput = 1;
        }

        //DarthVatorOutput repurposed for arm limit switches
        double DarthVatorOutput = RightJoystick.getRawAxis(1); //Moves the joystick based on Y value
        if (!upLimitSwitch.get()){ // If the forward limit switch is pressed, we want to keep the values between -1 and 0
            if(RightJoystick.getRawAxis(1) < 0){
                DarthVatorOutput = 0;
            }
            else{
                DarthVatorOutput = 1;
            }
        }
        else if(!downLimitSwitch.get()){ // If the reversed limit switch is pressed, we want to keep the values between 0 and 1
            if(RightJoystick.getRawAxis(1) >= 0){
                DarthVatorOutput = 0;
            }
            else{
                DarthVatorOutput = 1;
            }
        }
        else{
            DarthVatorOutput = 1;
        }
        
        if (RightJoystick.getRawButton(6)) {
            ArmDisplay = true;
            ArmTracer = 1;
            WristDisplay = false;
            WristTracer = 0;
            DarthVatorDisplay = false;
            DarthVatorTracer = 0;
            WheelieDisplay = false;
            WheelieTracer = 0;
        }
        else if(RightJoystick.getRawButton(7)){
            ArmDisplay = false;
            ArmTracer = 0;
            WristDisplay = true;
            WristTracer = 1;
            DarthVatorDisplay = false;
            DarthVatorTracer = 0;
            WheelieDisplay = false;
            WheelieTracer = 0;
        }
        else if(RightJoystick.getRawButton(11)){
            ArmDisplay = false;
            ArmTracer = 0;
            WristDisplay = false;
            WristTracer = 0;
            DarthVatorDisplay = true;
            DarthVatorTracer = 1;
            WheelieDisplay = false;
            WheelieTracer = 0;
        }
        else if(RightJoystick.getRawButton(10)){
            ArmDisplay = false;
            ArmTracer = 0;
            WristDisplay = false;
            WristTracer = 0;
            DarthVatorDisplay = false;
            DarthVatorTracer = 0;
            WheelieDisplay = true;
            WheelieTracer = 1;
        }
        else{

        }
        if(RightJoystick.getRawButton(4)){
            IntakeTracer = 1;
            IntakeTracerTracer = 0;
        }
        else if(RightJoystick.getRawButton(5)){
                IntakeTracer = -1;
                IntakeTracerTracer = 1;
            }
        else{
            if(IntakeTracerTracer == 1){
                IntakeTracer = 0;
            }
            else{
                IntakeTracer = 0.6; 
            }
            }

        DriveTrain.arcadeDrive(-LeftJoystick.getRawAxis(1), LeftJoystick.getRawAxis(2)*-0.8);
        IntakeDrive.tankDrive(IntakeTracer, IntakeTracer);
        LebronJames.tankDrive((ArmTracer*RightJoystick.getRawAxis(1)*DarthVatorOutput), WristTracer*RightJoystick.getRawAxis(1));
        NotArm.tankDrive((DarthVatorTracer*RightJoystick.getRawAxis(1)), WheelieTracer*RightJoystick.getRawAxis(1)*linearActuatorOutput*0.8);
        
        if (RightJoystick.getRawAxis(0)<0){
        	Red = (RightJoystick.getRawAxis(0)*-255);
        	Red = RedOut;
        }
        if (RightJoystick.getRawAxis(0)>0){
        	Blue = (RightJoystick.getRawAxis(0)*255);
        	Blue = BlueOut;
        }
        if (RightJoystick.getRawAxis(1)>0){
        	Green = (RightJoystick.getRawAxis(1)*255);
        	Green = GreenOut;
        }
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
    
}

