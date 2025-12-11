using UnityEngine;

// pure class
public class VehicleRobotDynamics
{

    private readonly VehicleParameters p;

    public VehicleRobotDynamics(VehicleParameters parameters)
    {
        p = parameters;
    }

    public float f0(VehicleRobotStateData s) => 1.0f;

    public float f1(VehicleRobotStateData s) => Mathf.Cos(s.theta1) * s.u1;

    public float f2(VehicleRobotStateData s) => Mathf.Sin(s.theta1) * s.u1;

    public float f3(VehicleRobotStateData s) => s.u2;

    public float f4(VehicleRobotStateData s) => Mathf.Tan(s.phi1) / p.GetL1() * s.u1;

    public float f5(VehicleRobotStateData s) => s.u3;

    public float f6(VehicleRobotStateData s) => Mathf.Sin(s.theta1 - s.theta3) / (p.GetL2() * Mathf.Cos(s.theta2 - s.theta3)) * s.u1;

    public float f7(VehicleRobotStateData s) => - (Mathf.Tan(s.phi2) * Mathf.Cos(s.theta2 - s.theta1)) / (p.GetL3() * Mathf.Cos(s.theta2 - s.theta3)) * s.u1;
}
