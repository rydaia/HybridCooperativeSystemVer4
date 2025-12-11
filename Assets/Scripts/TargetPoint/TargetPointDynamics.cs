using UnityEngine;

// pure class
public class TargetPointDynamics
{
    public float f0(TargetPointStateData s) => 1.0f;

    public float f1(TargetPointStateData s) => s.v1;

    public float f2(TargetPointStateData s) => Mathf.Cos(s.theta) * s.v1;

    public float f3(TargetPointStateData s) => Mathf.Sin(s.theta) * s.v1;

    public float f4(TargetPointStateData s) => s.v2;
}
