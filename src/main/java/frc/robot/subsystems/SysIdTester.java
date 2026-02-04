package frc.robot.subsystems;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class SysIdTester implements Subsystem {

    SysIdRoutine m_sysid;
    

    public SysIdTester() {
        m_sysid = new SysIdRoutine(new Config(), new Mechanism(null, this::LogSysId, this));
    }

    private void LogSysId(SysIdRoutineLog ALog) {
        System.out.println("SysIdLog");
    }

}
