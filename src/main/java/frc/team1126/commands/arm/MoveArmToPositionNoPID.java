package frc.team1126.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.Arm;
import frc.team1126.subsystems.sensors.Limelight;

public class MoveArmToPositionNoPID extends Command {
    private double m_targetAngle;
    private final Arm m_arm;
    private final Limelight m_Limelight;

    public MoveArmToPositionNoPID(Arm arm, Limelight limelight, double targetAngle) { // -3
        addRequirements(RobotContainer.m_arm, RobotContainer.m_limeLight);
        m_arm = arm;

        m_Limelight = limelight;
        m_targetAngle = targetAngle;
    }

    @Override
    public void execute() {
        // m_targetAngle = m_Limelight.getShootingAngle();
        // System.out.println("This angle " + m_targetAngle );
        double currentPitch = m_arm.getPitch();
        // double target = m_targetAngle;
        m_arm.moveArm(-.15);
       
    }

    @Override
    public void end(boolean interupted) {
        m_arm.moveArm(0);
    }

    @Override
    public boolean isFinished() {

        // if (m_arm.getPitch() >= m_targetAngle) {
        // return true;
        // }
        return false;
    }

}