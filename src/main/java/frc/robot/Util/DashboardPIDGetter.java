// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class DashboardPIDGetter {
    private final String m_prefix;
    private final PIDController m_pidController;
    private double m_P = 0;
    private double m_I = 0;
    private double m_D = 0;
    private double m_Target = 0;

    public DashboardPIDGetter(String prefix, double defaultP, double defaultI, double defaultD, double defaultTarget) {
        this(null, prefix, defaultP, defaultI, defaultD, defaultTarget);
    }

    public DashboardPIDGetter(PIDController pidController, String prefix, double defaultP, double defaultI, double defaultD, double defaultTarget) {
        m_pidController = pidController;
        m_prefix = prefix;
        m_P = defaultP;
        m_I = defaultI;
        m_D = defaultD;
        m_Target = defaultTarget;

        SmartDashboard.putNumber(m_prefix + " P", defaultP);
        SmartDashboard.putNumber(m_prefix + " I", defaultI);
        SmartDashboard.putNumber(m_prefix + " D", defaultD);
        SmartDashboard.putNumber(m_prefix + " Target", defaultTarget);
    }

    public double getDashboardP() {
        m_P = SmartDashboard.getNumber(m_prefix + " P", m_P);
        System.out.printf("[%s P value: %f]\n", m_prefix, m_P);
        return m_P;
    }

    public double getDashboardI() {
        m_I = SmartDashboard.getNumber(m_prefix + " I", m_I);
        System.out.printf("[%s I value: %f]\n", m_prefix, m_I);
        return m_I;
    }

    public double getDashboardD() {
        m_D = SmartDashboard.getNumber(m_prefix + " D", m_D);
        System.out.printf("[%s D value: %f]\n", m_prefix, m_D);
        return m_D;
    }

    public double getDashboardTarget() {
        m_Target = SmartDashboard.getNumber(m_prefix + " Target", m_Target);
        System.out.printf("[%s Target value: %f]\n", m_prefix, m_Target);
        return m_Target;
    }
    
    public void setPIDValuesFromDashboard() {
        m_pidController.setP(getDashboardP());
        m_pidController.setI(getDashboardI());
        m_pidController.setD(getDashboardD());
    }

    public void setTargetValueFromDashboard(){
        m_pidController.setSetpoint(getDashboardTarget());
    }

    public void setAllValuesFromDashboard(){
        setPIDValuesFromDashboard();
        setTargetValueFromDashboard();
    }
}
