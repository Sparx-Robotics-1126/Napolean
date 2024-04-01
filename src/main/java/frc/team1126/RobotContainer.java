// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126;

import java.io.File;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.team1126.commands.Limelight.LLRotationAlignCommand;
import frc.team1126.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final int m_rotationAxis = XboxController.Axis.kRightX.value;

    // public static final CANdleSubsystem m_candleSubsystem = new CANdleSubsystem();

    final static SendableChooser<Command> m_chooser = new SendableChooser<>();

    CommandXboxController m_driver = new CommandXboxController(Constants.GeneralConstants.DRIVER_CONTROLLER_ID);
    // CommandXboxController m_operator = new CommandXboxController(Constants.GeneralConstants.OPERATOR_CONTROLLER_ID);

    public static SwerveSubsystem m_swerve;

   
    public RobotContainer() {
      
        /* REGISTER PATHPLANNER COMMANDS HERE */
     

        //OTHER COMMANDS
        //NamedCommands.registerCommand("limelightTarget", new LLRotationAlignCommand(m_swerve).withTimeout(1.5));

        m_swerve = new SwerveSubsystem(
                new File(Filesystem.getDeployDirectory(), "swerve"));

        Command driveFieldOrientedAnglularVelocity = m_swerve.driveCommand(
                () -> MathUtil.clamp(MathUtil.applyDeadband(-m_driver.getLeftY(), .1), -1,
                        1),
                () -> MathUtil.clamp(MathUtil.applyDeadband(-m_driver.getLeftX(), .1), -1,
                        1),
                () -> -m_driver.getRightX());

        m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
       
        configureChooser();

        configureDriverBindings();

    }

    private void configureDriverBindings() {
        
        m_driver.leftTrigger().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
        m_driver.a().whileTrue(new LLRotationAlignCommand(m_swerve));
    }

   

    double getXSpeed() {
        int pov = m_driver.getHID().getPOV();
        double finalX;

        if (pov == 0)
            finalX = -0.05;
        else if (pov == 180)
            finalX = 0.05;
        else if (Math.abs(m_driver.getLeftY()) <= 0.1)
            finalX = 0.0;
        else
            finalX = m_driver.getLeftY() * 0.75 * (1.0 + m_driver.getLeftTriggerAxis());

        return finalX;
    }

    public double getYSpeed() {
        int pov = m_driver.getHID().getPOV();

        double finalY;
        if (pov == 270 || pov == 315 || pov == 225)
            finalY = -0.05;
        else if (pov == 90 || pov == 45 || pov == 135)
            finalY = 0.05;
        else if (Math.abs(m_driver.getLeftX()) <= 0.1)
            finalY = 0.0;
        else
            finalY = m_driver.getLeftX() * 0.75 * (1.0 + m_driver.getLeftTriggerAxis());

        return finalY;
    }

    public void configureChooser() {

        // autos using pathplanner
        m_chooser.setDefaultOption("Do Nothing", new WaitCommand(1));
        m_chooser.addOption("Leave Start Area", new PathPlannerAuto("LeaveStartingZone"));
        m_chooser.addOption("ShootAndMoveFromFront", new PathPlannerAuto("ShootMoveShoot"));

        m_chooser.addOption("shootfromAmpSide", new PathPlannerAuto("MoveFromAngleShoot"));
        m_chooser.addOption("shootFromSourceSide", new PathPlannerAuto("ShootFromRight"));
        m_chooser.addOption("3 NOTE AUTO", new PathPlannerAuto("3NoteAuto"));
        m_chooser.addOption("x tuning", new PathPlannerAuto("xTuningTest"));
        m_chooser.addOption("y tuning", new PathPlannerAuto("test"));
        m_chooser.addOption("angleTuning",new PathPlannerAuto("AngleTuning"));

        m_chooser.addOption("shootStage", new PathPlannerAuto("playoffs 1"));


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *\
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // swerve.zeroGyro();
        //
        // // m_driveSubsystem.setHeading(180);
        // Timer.delay(0.05);
        // // the command to be run in autonomous

        // return _chooser.getSelected();
        return m_chooser.getSelected();
        // return swerve.getAutonomousCommand(_chooser.getSelected().getName(), true);

    }

 

    public void EndGameRumble() {

        // if(DriverStation.getMatchTime() < 20 && DriverStation.getMatchTime() > 18) {
        //     m_candleSubsystem.setLEDState(LEDState.PURPLE);
        // }

        // if (DriverStation.getMatchTime() < SwerveConstants.ENDGAME_SECONDS 
        //         && DriverStation.getMatchTime() > SwerveConstants.STOP_RUMBLE_SECONDS) {
            
        //     m_driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
        //     m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);

        // } else {
        //     m_driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
        //     m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);

        // }
    }

    //method for operator to know whether or not the shooter is up to speed for any given distance
    public void upToSpeedRumble() {
        // if(m_shooter.isMotorUpToSpeed()) {
        //     m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble,1);
        // } 
        // m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble,0);
    }

}
