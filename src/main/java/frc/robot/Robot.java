// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
/* list of sensors to do because rehder:
gyroscope
accelerometer ?
encoders
limelight
ultrasonic

list of things to research:
apriltags
*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.opencv.objdetect.HOGDescriptor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.util.RootNameLookup;
import java.util.*;
//Todo: finetune PID constants using trial and error


// possible mechanics
// gyroscope: import com.ctre.phoenix.sensors.WPI_PigeonIMU;
// encoders: import edu.wpi.first.wpilibj.DigitalInput;
// limelight?
// servo: import edu.wpi.first.wpilibj.Servo;

// robot
// motors
// pneumatics
 
public class Robot extends TimedRobot {
  // private WPI_PigeonIMU gyro = new WPI_PigeonIMU(5);

  private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(1);
  private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(2);
  private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(3);
  private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(4);
  private final DifferentialDrive motors1 = new DifferentialDrive(leftMotor, rightMotor);
  private final DifferentialDrive motors2 = new DifferentialDrive(leftMotor2, rightMotor2);
  private final Joystick driverStick = new Joystick(2);
  Timer timer = new Timer();
  final ArrayList<Integer> ids = new ArrayList<Integer>();
  static final double kP = 0.001;
  static final double kI = 0.0;
  static final double kD = 0.0;
  AHRS gyro = new AHRS(); 
  // AHRS gyro;
  PIDController pid = new PIDController(kP, kI, kD);

  boolean hasScoredAutonomous = false;

  boolean driveTrainDirection = true;
  // intakeDirection = true: forwards
  // intakeDirection = false: backwards

  // create variable for distance between robot to target (horizontal)
  double distanceFromLimelightToGoalInches;
  int tagID;
  
  // pneumatics
  Solenoid Horn1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  Solenoid Horn2 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  Solenoid Horn3 = new Solenoid(PneumaticsModuleType.CTREPCM,2);
  Solenoid Horn4 = new Solenoid(PneumaticsModuleType.CTREPCM,3);
  Solenoid Horn5 = new Solenoid(PneumaticsModuleType.CTREPCM,4);
  Solenoid Horn6 = new Solenoid(PneumaticsModuleType.CTREPCM,5);
  Solenoid horns[] = {Horn1, Horn2, Horn3, Horn4, Horn5, Horn6};

  // temp
  boolean turning;
  boolean driving;
  boolean found;

  double a_turnValue;
  double a_driveValue;

  // encoder
  Encoder encoder = new Encoder(1, 0, false, Encoder.EncodingType.k2X);
  CANCoder cancoder = new CANCoder(1);

  // limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-rock");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry json = table.getEntry("json");
  ObjectMapper lightMapper = new ObjectMapper();
  double steeringSpeed = 0.6;
  double driveSpeed = 0.9;
  final double setPoint = 0;


  // read values periodically, use this code in autonomous and teleop
  /* double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0); */

  // ultrasonic http://www.maxbotix.com/documents/HRLV-MaxSonar-EZ_Datasheet.pdf
  AnalogInput mb1023 = new AnalogInput(0);
  // TODO - You will need to determine how to convert voltage to distance
  // (use information from the data sheet, or your own measurements)
  // ~4.88mV per 5mm
  double VOLTS_TO_DIST = .976;
  /* use this code in autonomous and teleop
  public static double getVoltage() {
    return mb1013.getVoltage();
  }
  public static double getDistance() {
    return getVoltage() * VOLTS_TO_DIST;
  }
  */
 
  @Override
  public void robotInit() {
    //initialize list of AprilTags to move towards
    //ids.add(0, 3);
    ids.add(0, 17);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    leftMotor2.setInverted(true);
    leftMotor.setInverted(true);
    //Streams camera to dashboard
    CameraServer.startAutomaticCapture(0);
    SmartDashboard.putBoolean("DriveTrain Direction", driveTrainDirection);
    // test code
    encoder.setDistancePerPulse(1./256.); // set distance values
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.sensorCoefficient = 2*Math.PI / 4096.0;
    config.unitString = "rad";
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    cancoder.configAllSettings(config);
    gyro = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50); 
    gyro.calibrate();
    

    //test code
  }
 
  public void robotPeriodic() {
    if (tv.getDouble(0) == 1) {
      double targetOffsetAngle_Vertical = ty.getDouble(0.0);

      // how many degrees back is your limelight rotated from perfectly vertical?
      double limelightMountAngleDegrees = 0;

      // distance from the center of the Limelight lens to the floor
      double limelightLensHeightInches = 3.25;

      // distance from the target to the floor
      double goalHeightInches = 29.5;

      double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

      //calculate distance
      distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
      SmartDashboard.putNumber("distance to goal", distanceFromLimelightToGoalInches);
    }
    SmartDashboard.putNumber("cancoder", cancoder.getPosition());

    SmartDashboard.putNumber("voltage", mb1023.getVoltage());
    SmartDashboard.putNumber("distance", (mb1023.getVoltage() * .976));
    SmartDashboard.putString("JSON Data", json.getString("No JSON Data"));
  }

  @Override
  public void teleopInit() {
    gyro.calibrate();
  }

  @Override
  public void teleopPeriodic() {
    // // reverse drivetrain button; button 4
    // if(driverStick.getRawButtonReleased(4) == true){
    //   if (driveTrainDirection == true) {
    //     driveTrainDirection = false;
    //   } else {
    //     driveTrainDirection = true;
    //   }System.out.println(json.getString("No JSON"));
    // reset 

    System.out.println(gyro.getAngle() + "gyroscope");
    
    if (driverStick.getRawButtonPressed(5)) {
      encoder.reset();
    }
    
    SmartDashboard.putNumber("DriveTrain Rotation", gyro.getAngle());
    //System.out.println("\n" + json.getString("No JSON Data"));
     try { JsonNode rootNode = lightMapper.readTree(json.getString(""));
          JsonNode idNode = rootNode.at("/Results/Fiducial/0/fID");//.path("Fiducial[0]").path("fID");
          tagID = idNode.asInt();
          System.out.println(tagID);
      //    System.out.println(tx.getDouble(0));
     } catch (Exception e) {System.out.println("Wierd JSON Bullshit");} 
    
   

    if (driverStick.getRawButtonPressed(2)) {
      if (driveTrainDirection == true) {
        driveTrainDirection = false;
        SmartDashboard.putBoolean("DriveTrain Direction", driveTrainDirection);
      } else {
        driveTrainDirection = true;
        SmartDashboard.putBoolean("DriveTrain Direction", driveTrainDirection);
      }
    }
    
    
    // SmartDashboard.putNumber("Compass Heading", (double)gyro.getRawGyroZ());
    //  SmartDashboard.putNumber("set point", setPoint);
    //  SmartDashboard.putNumber("PID value", pid.calculate((double)gyro.getRawGyroZ(), setPoint));
    SmartDashboard.putNumber("gyro Z value", gyro.getRawGyroZ());
    SmartDashboard.putNumber("gyro X value", gyro.getRawGyroX());
    SmartDashboard.putNumber("gyro Y value", gyro.getRawGyroY());
    SmartDashboard.putNumber("gyro YAW value", gyro.getYaw());

    double driveAmount = driverStick.getZ() + (driverStick.getX() * 0.5) * steeringSpeed;

    if (driveTrainDirection == true){
      motors1.arcadeDrive(driverStick.getY() * driveSpeed, -driveAmount);
      motors2.arcadeDrive(driverStick.getY() * driveSpeed, -driveAmount);
    } else {
      motors1.arcadeDrive(-driverStick.getY() * driveSpeed, driveAmount);
      motors2.arcadeDrive(-driverStick.getY() * driveSpeed, driveAmount);
    }

    // pneumatics example
    // if (driverStick.getRawButtonPressed(7)) {
    //   Horn1.set(true);
    // }
    // if (driverStick.getRawButtonReleased(7)) {
    //   Horn1.set(false);
    // }
    // if (driverStick.getRawButtonPressed(8)) {
    //   Horn2.set(true);
    // }
    // if (driverStick.getRawButtonReleased(8)) {
    //   Horn2.set(false);
    // }
    // if (driverStick.getRawButtonPressed(9)) {
    //   Horn3.set(true);
    // }
    // if (driverStick.getRawButtonReleased(9)) {
    //   Horn3.set(false);
    // }
    // if (driverStick.getRawButtonPressed(10)) {
    //   Horn4.set(true);
    // }
    // if (driverStick.getRawButtonReleased(10)) {
    //   Horn4.set(false);
    // }
    // if (driverStick.getRawButtonPressed(11)) {
    //   Horn5.set(true);
    // }
    // if (driverStick.getRawButtonReleased(11)) {
    //   Horn5.set(false);
    // }
    // if (driverStick.getRawButtonPressed(12)) {
    //   Horn6.set(true);
    // }
    // if (driverStick.getRawButtonReleased(12)) {
    //   Horn6.set(false);
    // }

    // motor (percent output) example
    // if (intakeStatus == true) {
    //   intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
    // } else {
    //   intakeMotor.set(ControlMode.PercentOutput, 0);
    // }
  }
 
  @Override
  public void autonomousInit() {
    // timer.reset();
    // timer.start();
    // if(timer.get()>6.75) {
    // }
    gyro.calibrate();
    motors1.setDeadband(0.0001);
    motors2.setDeadband(0.0001);
    pid.setTolerance(1);
    pid.setSetpoint(90.0);
    pid.enableContinuousInput(-180, 180);
    // rhythm(1, 100, 100, 10);
  }
  
  public void hornPlayNote_async(int hornNo, long time_ms){
    Thread playThread = new Thread(() -> {
      horns[hornNo - 1].set(true);
      try {Thread.sleep(time_ms); } catch (Exception e) {System.out.println("I lay awake, thinking about all the things I had done wrong during the day.");}
      horns[hornNo - 1].set(false);
    });
    playThread.start();
  }

  public void hornPlayNote(int hornNo, long time_ms){
  horns[hornNo -1].set(true);
  try {Thread.sleep(time_ms); } catch (Exception e) {System.out.println("I lay awake, thinking about all the things I had done wrong during the day.");}
  horns[hornNo -1].set(false);
  } 

  public void rhythm(int hornNo, long time_on, long time_off, int iter){
    Thread rhythmThread = new Thread(() -> {
    for(int i = 0; i <= iter; i++){
      hornPlayNote(hornNo, time_on);
      try {Thread.sleep(time_off); } catch (Exception e) {System.out.println("I lay awake, thinking about all the things I had done wrong during the day.");}
    }
    });
    rhythmThread.start();
  }
// Song is a 2d array of longs, first [] defines the current note to play second [] is the info of the note. song[i][0] is the horn to play note through (If this is zero, the Program sleeps without playing anything) and song[i][1] is the time to play it for.
  // public void playSong(long[][] song) {
  //   for(int i = 0; i <= song.length; i++){
  //     if(song[i][0] == 0) {
  //       try {Thread.sleep(song[i][1]); } catch (Exception e) {System.out.println("I lay awake, thinking about all the things I had done wrong during the day.");}
  //       continue;

  //     } else if (song[i][0] > 6 || song[i][0] < 0){
  //       System.out.println("Invalid Horn Number at note" + i);
  //       continue;

  //     }
  //     hornPlayNote((int)song[i][0], song[i][1]);


  //   }




  // }


  @Override
  public void autonomousPeriodic() {
    // timer.start();
    // // if(encoder.getDistance() < 5) {
    // //   motors1.arcadeDrive(.5, 0);
    // //   motors2.arcadeDrive(.5, 0);
    // // } else {
    // //     motors1.arcadeDrive(0, 0);
    // //     motors2.arcadeDrive(0, 0);
    // // }
    // // if (tv.getDouble(0) == 1) {
    // //   motors1.arcadeDrive(.5, 0);
    // //   motors2.arcadeDrive(.5, 0);
    // // } else {
    // //   motors1.arcadeDrive(0, 0);
    // //   motors2.arcadeDrive(0, 0);
    // // }
    // // if target found, if no target found
    // if (tv.getDouble(0) == 1) {
    //   found = true;
    //   if (tx.getDouble(0) > .5) {
    //     System.out.println("turning");
    //     turning = true;
    //     driving = false;
    //     leftMotor.set(ControlMode.PercentOutput, -(tx.getDouble(0))/54);
    //     leftMotor2.set(ControlMode.PercentOutput, -(tx.getDouble(0))/54);
    //     rightMotor.set(ControlMode.PercentOutput, (tx.getDouble(0))/54);
    //     rightMotor2.set(ControlMode.PercentOutput, (tx.getDouble(0))/54);
    //   } else if (tx.getDouble(0) < -.5) {
    //     System.out.println("turning");
    //     turning = true;
    //     driving = false;
    //     leftMotor.set(ControlMode.PercentOutput, -(tx.getDouble(0))/54);
    //     leftMotor2.set(ControlMode.PercentOutput, -(tx.getDouble(0))/54);
    //     rightMotor.set(ControlMode.PercentOutput, (tx.getDouble(0))/54);
    //     rightMotor2.set(ControlMode.PercentOutput, (tx.getDouble(0))/54);
    //   } else {
    //     driving = true;
    //     System.out.println("yay");
    //     if (distanceFromLimelightToGoalInches > 24.5) {
    //       System.out.println("Going Forward");
    //       driving = true;
    //       turning = false;
    //       if (distanceFromLimelightToGoalInches > 48) {
    //         leftMotor.set(ControlMode.PercentOutput, 1);
    //         leftMotor2.set(ControlMode.PercentOutput, 1);
    //         rightMotor.set(ControlMode.PercentOutput, 1);
    //         rightMotor2.set(ControlMode.PercentOutput, 1);
    //       } else {
    //         leftMotor.set(ControlMode.PercentOutput, distanceFromLimelightToGoalInches/48);
    //         leftMotor2.set(ControlMode.PercentOutput, distanceFromLimelightToGoalInches/48);
    //         rightMotor.set(ControlMode.PercentOutput, distanceFromLimelightToGoalInches/48);
    //         rightMotor2.set(ControlMode.PercentOutput, distanceFromLimelightToGoalInches/48);
    //       }
    //     } else if (distanceFromLimelightToGoalInches < 23.5) {
    //       driving = true;
    //       turning = true;
    //       System.out.println("Going Backward");
    //       leftMotor.set(ControlMode.PercentOutput, -distanceFromLimelightToGoalInches/48);
    //       leftMotor2.set(ControlMode.PercentOutput, -distanceFromLimelightToGoalInches/48);
    //       rightMotor.set(ControlMode.PercentOutput, -distanceFromLimelightToGoalInches/48);
    //       rightMotor2.set(ControlMode.PercentOutput, -distanceFromLimelightToGoalInches/48);
    //     }
    //     else {
          

    //     }
    //   }
    // } else {
    //   System.out.println("Not found");
    //   found = false;
    //   if (timer.get() < .5) {
    //     motors1.arcadeDrive(0, -.5);
    //     motors2.arcadeDrive(0, -.5);
    //   } 
    //   if (timer.get() < 1.5 && timer.get() >= .5) {
    //     motors1.arcadeDrive(0, 0);
    //     motors2.arcadeDrive(0, 0);
    //   }
    //   if (timer.get() >= 1.5) {
    //     timer.reset();
    //   }
    // }
    try { JsonNode rootNode = lightMapper.readTree(json.getString(""));
          JsonNode idNode = rootNode.at("/Results/Fiducial/0/fID");//.path("Fiducial[0]").path("fID");
          tagID = idNode.asInt();
          System.out.println(tagID);
     } catch (Exception e) {System.out.println("Wierd JSON Bullshit");} 
    

     SmartDashboard.putNumber("gyroscope", gyro.getYaw());
     SmartDashboard.putNumber("set point", setPoint);
    //  SmartDashboard.putNumber("PID value", pid.calculate(gyro.getAngle(), setPoint));
    //  // P controller output: kP * error (90-gyro.getAngle()); reverse (make negative output)
     // setpoint = -90 + gyro.getAngle()
     /* setpoint of the PID is 90 degrees
      * input of PID is gyro angle
      output of the PID is 1 side of the drivatrain, other side run at the exact opposite of the normal output
      */

     // kP = 1/90
    //  if (!pid.atSetpoint()){
          // Find the heading error; setpoint is 90
          double angle = gyro.getYaw();
      // if (gyro.getYaw() < 180) {
      //   angle = 180-gyro.getYaw();
      //   System.out.println(angle);
      // }


      //motors1.arcadeDrive(0, pid.calculate(gyro.getYaw(), 90));
      //motors2.arcadeDrive(0, pid.calculate(gyro.getYaw(), 90));
      while (!pid.atSetpoint()) {
        motors1.arcadeDrive(0, pid.calculate(angle, 90));
        motors2.arcadeDrive(0, pid.calculate(angle, 90));
        System.out.println("no");
        if (pid.atSetpoint()) {
          leftMotor.set(ControlMode.PercentOutput, 0);
          leftMotor2.set(ControlMode.PercentOutput, 0);
          rightMotor.set(ControlMode.PercentOutput, 0);
          rightMotor2.set(ControlMode.PercentOutput, 0);
          System.out.println("slay");
        }
      }
      // if (pid.atSetpoint()) {
      //   atSetpoint = true;
      //   leftMotor.set(ControlMode.PercentOutput, 0);
      //     leftMotor2.set(ControlMode.PercentOutput, 0);
      //     rightMotor.set(ControlMode.PercentOutput, 0);
      //     rightMotor2.set(ControlMode.PercentOutput, 0);
      //  } else if (atSetpoint = false) {
      //   motors1.arcadeDrive(0, pid.calculate(angle, 90));
      //   motors2.arcadeDrive(0, pid.calculate(angle, 90));
      //   System.out.println("no");
      //  }

    // Turns the robot to face the desired direction
    // if (pidValue == 0) {
    //   motors1.tankDrive(kP * error, -kP * error);
    //   motors2.tankDrive(kP * error, -kP * error);
    // } else {
    //   motors1.arcadeDrive(0, 0);
    //   motors2.arcadeDrive(0, 0);
    // }
    //  }
    
    /*if (tv.getDouble(0) == 1) {
    //   double xAngleOffset = tx.getDouble(0);
    //   motors1.arcadeDrive(0, pid.calculate(gyro.getAngle(), setPoint));
      //double RobotVerticalAngle = ty.getDouble(0);
      // if (RobotVerticalAngle > 19.5) {
      //   leftMotor.set(ControlMode.PercentOutput, RobotVerticalAngle);
      //   leftMotor2.set(ControlMode.PercentOutput, RobotVerticalAngle);
      //   rightMotor.set(ControlMode.PercentOutput, RobotVerticalAngle);
      //   rightMotor2.set(ControlMode.PercentOutput, RobotVerticalAngle);
      // } else if(RobotVerticalAngle < 18.5) {
      //   leftMotor.set(ControlMode.PercentOutput, 0);
      //   leftMotor2.set(ControlMode.PercentOutput, 0);
      //   rightMotor.set(ControlMode.PercentOutput, 0);
      //   rightMotor2.set(ControlMode.PercentOutput, 0);
      // } else {
      //   leftMotor.set(ControlMode.PercentOutput, 0);
      //   leftMotor2.set(ControlMode.PercentOutput, 0);
      //   rightMotor.set(ControlMode.PercentOutput, 0);
      //   rightMotor2.set(ControlMode.PercentOutput, 0);
      // }
      // a_turnValue = -tx.getDouble(0)/54;
      // if (ty.getDouble(0) > 19.25) {
        // a_driveValue = (ty.getDouble(0)-19.25)/5.35;
      } else if(ty.getDouble(0) < 18.75) {
        // a_driveValue = -(9.25+(-(ty.getDouble(0)-9.25)))/18.5;
      } else {
        // a_driveValue = 0;
        // arm code goes here
      } 
      if (a_driveValue > 1) {
        a_driveValue = 1;
      } else if(a_driveValue < -1) {
        a_driveValue = -1;
      } 
      SmartDashboard.putNumber("drive", a_driveValue);
      SmartDashboard.putNumber("turn", a_turnValue);
      System.out.println("turn: "+a_turnValue);
      System.out.println("drive: "+a_driveValue);
      motors1.arcadeDrive(a_driveValue, a_turnValue, false);
      motors2.arcadeDrive(a_driveValue, a_turnValue, false);
      // leftMotor.set(ControlMode.PercentOutput, a_driveValue-(tx.getDouble(0))/54);
      // leftMotor2.set(ControlMode.PercentOutput, a_driveValue-(tx.getDouble(0))/54);
      // rightMotor.set(ControlMode.PercentOutput, a_driveValue+(tx.getDouble(0))/54);
      // rightMotor2.set(ControlMode.PercentOutput, a_driveValue+(tx.getDouble(0))/54);
    } else {
      found = false;
      if (timer.get() < .5) {
        motors1.arcadeDrive(0, -.5);  //.5
        motors2.arcadeDrive(0, -.5);   //.5
      } 
      if (timer.get() < 1.5 && timer.get() >= .5) {
        motors1.arcadeDrive(0, 0);
        motors2.arcadeDrive(0, 0);
      }
      if (timer.get() >= 1.5) {
        timer.reset();
      }
    } */
  }
}