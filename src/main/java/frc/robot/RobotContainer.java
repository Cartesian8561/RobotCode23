// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.Routines.AutonomousRoutines;
import frc.robot.autonomous.Trajectories.AutonomousTrajectories;
import frc.robot.Constants.ArmPoseConstants;
import frc.robot.Constants.IntakeStabilizeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.BundledCommand;
import frc.robot.autonomous.CartesianRamseteClass;
import frc.robot.commands.ArmCommands.CloseArm;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.OpenArm;
import frc.robot.commands.ChargingStationCommand.BalanceCommand;
import frc.robot.commands.ChargingStationCommand.ChargeCommand;
import frc.robot.commands.GamePieceCommands.GetConeCommand;
import frc.robot.commands.GamePieceCommands.HoldCube;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeAngleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
 
public class RobotContainer {
  // The robot's subsystems
  private final Drivetrain m_robotDrive = new Drivetrain();
  private final AutonomousTrajectories m_autoTrajectories = new AutonomousTrajectories(m_robotDrive);
  private final CartesianRamseteClass m_ramsete = new CartesianRamseteClass(m_robotDrive);
  private final AutonomousRoutines m_autos = new AutonomousRoutines(m_autoTrajectories, m_ramsete, m_robotDrive);
  private final ArmSubsystem armSub = new ArmSubsystem();
  private final OpenArm openArmTest = new OpenArm(armSub, ArmPoseConstants.armGroundPose); //Grid 4.7, !Substation 0.4!
  private final CloseArm closeArmTest = new CloseArm(armSub, -0.2);
  private final IntakeSubsystem intakeSub = new IntakeSubsystem();
  private final IntakeAngleSubsystem intakeAngleSub = new IntakeAngleSubsystem();
  //private final OpenIntake openIntakeTest = new OpenIntake(intakeAngleSub, -0.65, IntakeStabilizeConstants.intakeGridConeVolt); //annen2
  //private final OpenIntake testIntake = new OpenIntake(intakeAngleSub, -0.24, 1.1);
  private final ExtendArm extendArmTest = new ExtendArm(armSub, -28.0);
  private final ExtendArm retractArmTest = new ExtendArm(armSub, -3.0); 
  private final BundledCommand commands = new BundledCommand(m_autos, armSub, intakeSub, intakeAngleSub, IntakeStabilizeConstants.intakeGridConeVolt);
  private final HoldCube holdCube = new HoldCube(intakeSub);
  private final ChargeCommand balancer = new ChargeCommand(m_robotDrive);
  //private final IntakeStabilize stabilize = new IntakeStabilize(intakeSub);

  private final GetConeCommand coneCommand = new GetConeCommand(intakeSub);
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private static boolean isTank = false;
  private static int reverser = -1;
  private static boolean isStable = true;
  private static double setPoint = 0.0;
  //private static DigitalOutput digitalOutput = new DigitalOutput(2);
  //private static DigitalOutput cubeConeSignal = new DigitalOutput(3);

  private int driveState = 2;


  // The driver's controllers
  Joystick m_rightStick = new Joystick(OIConstants.kRightJoystick);
  Joystick m_leftStick = new Joystick(OIConstants.kLeftJoystick);
  XboxController m_blueXbox = new XboxController(OIConstants.kBlueXboxStick);
  XboxController m_grayXbox = new XboxController(OIConstants.kGrayXboxStick);

  public void updateOdometry(){
    m_robotDrive.updateOdometry();
  }
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //if(DriverStation.getAlliance() == Alliance.Red) digitalOutput.set(true);
    //else digitalOutput.set(false);

    SmartDashboard.putNumber("setPointHere", setPoint);
    armSub.updateEncoder();
    
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(() -> m_robotDrive.tankOrArcadeDrive(m_rightStick.getY()/getDriveSpeed(), m_rightStick.getX()/getDriveSpeed(), m_leftStick.getY()/getDriveSpeed(), m_rightStick.getY()/getDriveSpeed(), isTank, reverser), m_robotDrive));

    armSub.setDefaultCommand( //kol angle + kol uzunluk
      new RunCommand(() -> armSub.setArm(-m_blueXbox.getLeftY() / 1.5, m_blueXbox.getRawAxis(3)  / 1.7), armSub));

    //intakeSub.setDefaultCommand(new RunCommand(() -> stabilize.runPID(setPoint), intakeSub));
    
    m_autoChooser.setDefaultOption("Test", m_autos.gamePiece3Command());
    m_autoChooser.addOption("koni", m_autos.gamePiece3Command());
    m_autoChooser.addOption("ananiskim", m_autos.leftGrid());

    SmartDashboard.putData(m_autoChooser);
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */

  private void configureButtonBindings() {

    new JoystickButton(m_leftStick, 11)
    .whileTrue(commands.setAutoCubeGridPosition())
    //.whileTrue(commands.setStartingPosition())
    .onFalse(new InstantCommand(() -> intakeSub.setBothIntakeMotorVolts(0.0), intakeSub));
    
    new JoystickButton(m_leftStick, 12)
    .whileTrue(commands.setStartingPosition())
    .onFalse(new InstantCommand(() -> intakeSub.setBothIntakeMotorVolts(0.0), intakeSub));
    

    new JoystickButton(m_leftStick, 10)
    .whileTrue(balancer)
    .onFalse(new InstantCommand(() -> m_robotDrive.tankDriveVolts(0, 0)));

    new JoystickButton(m_rightStick, 1)//TANK ARCADE GECIS 
    .onTrue(new InstantCommand(() -> changeDriveState()));

    new JoystickButton(m_rightStick, 2)//ROBOT YONU DEGISTIR
    .onTrue(new InstantCommand(() -> reverseChanger()));

    new POVButton(m_rightStick, 0)
    .onTrue(new InstantCommand(() -> changeDriveSpeed(false)));

    new POVButton(m_rightStick, 180)
    .onTrue(new InstantCommand(() -> changeDriveSpeed(true)));



    new JoystickButton(m_blueXbox, 3)
    .whileTrue(commands.setCubeGridPosition())
    .onFalse(new RunCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeAngleSub));

    new JoystickButton(m_blueXbox, 1)
    .whileTrue(commands.setConeMidPosition())
    .onFalse(new RunCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeAngleSub));

    new JoystickButton(m_blueXbox, 4)
    .whileTrue(commands.setConeGridPosition())
    .onFalse(new RunCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeAngleSub));
    
    new JoystickButton(m_blueXbox, 6)
    .whileTrue(commands.setConeGroundPosition())
    .onFalse(new RunCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeAngleSub));

/* 
    new JoystickButton(m_blueXbox, 2)
    .whileTrue(//commands.setCubeMidPosition)
    null)
    .onFalse(new RunCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeAngleSub));

    new JoystickButton(m_blueXbox, 10)
    .whileTrue(//commands.setCollisionPose)
    null)
    .onFalse(new RunCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeAngleSub));
*/
    new JoystickButton(m_blueXbox, 9)
    .whileTrue(commands.setDrivePose())
    .onFalse(new RunCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeAngleSub));

    new JoystickButton(m_blueXbox, 8) //INTAKE POZITIF DONDUR (R2)
        .whileTrue(new RunCommand(() -> intakeSub.setBothIntakeMotorVolts(2.5), intakeSub))
        .onFalse(new RunCommand(() -> intakeSub.setBothIntakeMotorVolts(0.0), intakeSub));

    new JoystickButton(m_blueXbox, 7) //INTAKE NEGATIF DONDUR (L2)
        .whileTrue(new RunCommand(() -> intakeSub.setBothIntakeMotorVolts(-1.5), intakeSub))
        .onFalse(new InstantCommand(() -> intakeSub.setBothIntakeMotorVolts(0.0), intakeSub));

    new POVButton(m_blueXbox, 0) //INTAKE YUKARIYA
        .whileTrue(new RunCommand(() -> intakeAngleSub.setIntakeVolts(10.0), intakeSub))
        .onFalse(new RunCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeSub));

    new POVButton(m_blueXbox, 180) //INTAKE ASAGIYA
        .whileTrue(new RunCommand(() -> intakeAngleSub.setIntakeVolts(-10.0), intakeSub))
        .onFalse(new RunCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeSub));

  }

  public Drivetrain getRobotDrive() {
    return m_robotDrive;
  }

  /** Zeros the outputs of all subsystems. */
  public void zeroDriveOutputs() {
    m_robotDrive.tankDriveVolts(0, 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_robotDrive.resetVision();
    m_robotDrive.resetGyro();
    return m_autoChooser.getSelected();
  }

  private double getDriveSpeed(){
    double a = 1 + (driveState * 0.25);
    SmartDashboard.putNumber("MotorSpeedDivider", a);
    return a;
  }

  private void changeDriveSpeed(boolean isDecreasing){
    if (isDecreasing){
      if (driveState == 3){
        driveState = 0;
      }else{
      driveState++;        
      }
    }else{
      if(driveState == 0){
        driveState = 3;
      }else{
        driveState--;
      }
    }
  }

  private void changeDriveState(){
    isTank = !isTank;
  }

  private void reverseChanger(){
    reverser *= -1;
  }

  public void setArmStable(){
    isStable = !isStable;
  }

  public void armVoltsZero(){
    if(isStable) armSub.setAngleMotorVolts(-2.0 * armSub.getScaler());
    else armSub.setAngleMotorVolts(0.0);
  }

  public void increaseSetPoint(){
    setPoint += 1.0;
  }

  public void decreaseSetPoint(){
    setPoint -= 1.0;
  }
  public void updateEncoder(){
    armSub.updateEncoder();
  }
} 