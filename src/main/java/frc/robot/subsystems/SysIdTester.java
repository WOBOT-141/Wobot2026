package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class SysIdTester {
    

    public SysIdTester() {
        SysIdRoutine _sysid = new SysIdRoutine(new Config(), new Mechanism(null, null, null));
    }


}
