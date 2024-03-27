// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPoseConstants;
import frc.robot.Constants.AutoPoseConstants;
import frc.robot.Constants.IntakePoseConstants;
import frc.robot.Constants.IntakeStabilizeConstants;
import frc.robot.autonomous.Routines.AutonomousRoutines;
import frc.robot.commands.ArmCommands.CloseArm;
import frc.robot.commands.ArmCommands.ExtendArm;
import frc.robot.commands.ArmCommands.LowerOpenIntake;
import frc.robot.commands.ArmCommands.OpenArm;
import frc.robot.commands.ArmCommands.OpenIntake;
import frc.robot.commands.GamePieceCommands.GetConeCommand;
import frc.robot.commands.GamePieceCommands.HoldCube;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeAngleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class BundledCommand {

    private ArmSubsystem armSub;
    private IntakeAngleSubsystem intakeAngleSub;
    private IntakeSubsystem intakeSub;
    private double stabilizeVolts;
    private AutonomousRoutines auto;

    public BundledCommand(AutonomousRoutines autonomousRoutines ,ArmSubsystem armSubsystem,IntakeSubsystem intakeSubsystem, IntakeAngleSubsystem intakeAngleSubsystem, double intakeStabilizerVolts){
        auto = autonomousRoutines;
        armSub = armSubsystem;
        stabilizeVolts = intakeStabilizerVolts;
        intakeAngleSub = intakeAngleSubsystem;
        intakeSub = intakeSubsystem;
    }

    public SequentialCommandGroup setConeGroundPosition(){
        return (new SequentialCommandGroup(
            new InstantCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeAngleSub),
            new ExtendArm(armSub, -3.0),
            new ParallelCommandGroup(new CloseArm(armSub, ArmPoseConstants.armGroundPose), new LowerOpenIntake(intakeAngleSub, IntakePoseConstants.intakeGroundConePose, stabilizeVolts))
          //new OpenArm(armSub, ArmPoseConstants.armGroundPose),
          //new OpenIntake(intakeSub, IntakePoseConstants.intakeGroundConePose)
        ));
    }

    public SequentialCommandGroup setCubeGroundPosition(){
        return (new SequentialCommandGroup(
            new ExtendArm(armSub, -3.0),
            new InstantCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeAngleSub),
          //new ParallelCommandGroup(new OpenArm(armSub, 0), new OpenIntake(intakeSub, 0)),
          
          new CloseArm(armSub, ArmPoseConstants.armGroundPose),
          new LowerOpenIntake(intakeAngleSub, IntakePoseConstants.intakeGroundCubePose, stabilizeVolts)));
    }
    

  
    public SequentialCommandGroup setConeGridPosition(){
        return (new SequentialCommandGroup(
            new InstantCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeAngleSub),
            new SequentialCommandGroup(new OpenArm(armSub, ArmPoseConstants.armGridPose), new ExtendArm(armSub, -20)),
            new OpenIntake(intakeAngleSub, -0.735, IntakeStabilizeConstants.intakeGridConeVolt)));
    }


    
    public SequentialCommandGroup setConeMidPosition(){
        return (new SequentialCommandGroup(
            new InstantCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeAngleSub),
            new SequentialCommandGroup(new OpenArm(armSub, 3.8), new ExtendArm(armSub, -5)),
            new OpenIntake(intakeAngleSub, -0.63, IntakeStabilizeConstants.intakeGridConeVolt)));
    }

    public SequentialCommandGroup setCubeGridPosition(){
        return (new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new HoldCube(intakeSub),
                    new SequentialCommandGroup(
                        new InstantCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeAngleSub),
                        new OpenArm(armSub, ArmPoseConstants.armGridPose),
                        new ExtendArm(armSub, -28))),
                new OpenIntake(intakeAngleSub, IntakePoseConstants.intakeGridCubePose, stabilizeVolts)
            ));

    }

    public SequentialCommandGroup getConeCommands(){
        return (new SequentialCommandGroup(
            setConeGroundPosition(),
            new GetConeCommand(intakeSub)
        ));
    }

    public SequentialCommandGroup setDrivePose(){
        return (new SequentialCommandGroup(
            new InstantCommand(() -> intakeAngleSub.setIntakeVolts(0.0), intakeAngleSub),
            new SequentialCommandGroup(
                new ExtendArm(armSub, -0.8),
                new CloseArm(armSub, -0.5)
            ),
            new OpenIntake(intakeAngleSub, 0.0, stabilizeVolts)
        ));
    }

    public SequentialCommandGroup getCubeCommand(){
        return (new SequentialCommandGroup(
            setCubeGroundPosition(),
            new RunCommand(() -> intakeSub.setBothIntakeMotorVolts(1.5), intakeSub)
        ));
    }

    

    public ParallelCommandGroup setStartingPosition(){
        return new ParallelCommandGroup(
            new OpenArm(armSub, ArmPoseConstants.armStartingPose),
            new OpenIntake(intakeAngleSub, IntakePoseConstants.intakeStartingPosition, stabilizeVolts));
    }
    //---------------------------AUTONOMOUS COMMANDS--------------------------------//

    public SequentialCommandGroup getTestAutonom(){
        return (new SequentialCommandGroup(
            //auto.testAuto(),
            getCubeCommand(),
            setDrivePose(),
            //auto.goTo3(),
            setCubeGroundPosition(),
            setDrivePose(),
            //auto.goToCube(),
            setConeGridPosition()
        ));
    }

    
    public ParallelCommandGroup setAutoCubeGridPosition(){
        return (new ParallelCommandGroup(
            new SequentialCommandGroup(new ParallelRaceGroup(new HoldCube(intakeSub),
                                                            new SequentialCommandGroup(new OpenArm(armSub, AutoPoseConstants.cubeHighGrid),new ExtendArm(armSub, -7.0))),
                    new RunCommand(() -> intakeSub.setBothIntakeMotorVolts(2.5), intakeSub)),
            new OpenIntake(intakeAngleSub, AutoPoseConstants.intakeCubeHighGrid, stabilizeVolts)
        ));
        
    }

}

