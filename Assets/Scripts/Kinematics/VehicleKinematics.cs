using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Profiling;


public class VehicleKinematics : MonoBehaviour
{

    static ProfilerMarker VehicleKinematicsMarker =
        new ProfilerMarker("VehicleKinematics.CalculationVehicleKinematics()");


    public VehicleRobotState vehicleRobotState;
    private TargetPointState targetPointState;
    private VehicleParameters vehicleRobotParms;
    private BsplineGeometry bsplineGeometry;
    private PathKinematics pathKinematics;    


    [Header("フィードバック制御ゲイン")]
    public float p31 = -18.0f;// 0.75
    public float p32 = -108.0f; //0.1865
    public float p33 = -216.0f; //
    public float k1 = 6.0f;
    public float k2 = 6.0f;
    public float k3 = 6.0f;
    public float K = 2.0f;

    [Header("制御パラメータ 参照")]

    private float L1;       // 車軸間距離の長さ[m]
    private float L2;      // 車軸間距離の長さ[m]
    private float L3;      // 車軸間距離の長さ[m]

    public float z31;
    public float z32, z33;
    // z32, z33;
    public float z21, z22;

    public float w1, w2, w3;

    public float _thetaP2_1, _thetaP2_2, _thetaP2_3;
    public float _thetaP2d_1, _thetaP2d_2, _thetaP2d_3;


    // 制御入力
    [SerializeField] private float u1;
    [SerializeField] private float u2;
    [SerializeField] private float u3;
    [SerializeField] private float thetaP1;
    [SerializeField] private float thetaP2;
    [SerializeField] private float thetaP3;

    public float u4; // 


    public float _cs1;
    public float _squaredCs1;
    public float _cs2;

    public float _d1c1ds11;
    public float _d2c1ds12;

    public float _d1c2ds21;
    public float _d2c2ds22;
    public float _squaredD1c1ds11;

    public float _d1;
    public float _squaredD1;

    public float _phi1;
    public float _phi2;

    // public float thetaP1;
    // public float thetaP2;
    // public float thetaP3;
    public float _thetaP2d;

    public float L1f1h2;
    public float L2f1h2;
    public float L3f1h2;
    
    public float L1f1h3;
    public float L2f1h3;
    public float L3f1h3;

    public float Lf2L2f1h2;
    public float Lf3L2f1h2;
    public float Lf3L2f1h3;

    public float _cosThetaP1;
    public float _sinThetaP1;
    public float _tanThetaP1;
    public float _secThetaP1; // secThetaP1

    public float _squaredSecThetaP1;
    public float _cubedSecThetaP1;
    public float _squaredTanThetaP1;

    public float _tanPhi1;
    public float _secPhi1;
    public float _squaredSecPhi1;

    public float _secPhi2; 
    public float _tanPhi2;
    public float _squaredSecPhi2;

    public float _cosThetaP2MinusThetaP3;
    public float _secThetaP2MinusThetaP3;
    public float _tanThetaP2MinusThetaP3;
    public float _squaredSecThetaP2MinusThetaP3;
    public float _cubedSecThetaP2MinusThetaP3;
    public float _squaredTanThetaP2MinusThetaP3;

    public float _sinThetaP1MinusThetaP3;
    public float _cosThetaP1MinusThetaP3;
    public float _squaredSinThetaP1MinusThetaP3;

    public float _cosThetaP1MinusThetaP2;
    public float _sinThetaP1MinusThetaP2;

    public float _formulaOf1MinusCs1MulD1;
    public float _squaredFormulaOf1MinusCs1MulD1;
    public float _cubedFormulaOf1MinusCs1MulD1;

    public float _formulaOfCs1MulSecThetaP1;
    public float _formulaOfCs1MulCosThetaP1;

    public float _squaredL2;

    public float _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3;
    public float _formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3;
    public float _formulaOfCosThetaP1MinusThetaP2MulSecThetaP2MinusThetaP3;
    public float _formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1;
    public float _formulaOfCs1MulCosThetaP1MulD1MulD1c1ds11DevSquaredFormulaOf1MinusCs1MulD1;
    public float _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2;
    public float _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1;
    public float _formulaOfFormulaOfCosThetaP1MulD1c1ds11DevFormulaOf1MinusCs1MulD1;
    public float _formulaOfCosThetaP1MulD1c1ds11;
    public float _formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1;
    public float _formulaOfD1MulSecThetaP1;
    public float _formulaTanPhi1DevL1;
    public float _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1;
    public float _formulaOfCubedSecThetaP2MinusThetaP3MulSinThetaP1MinusThetaP3;

    // public VehicleKinematics(
    //     TargetPointState TPstate,
    //     VehicleRobotState robot, 
    //     VehicleParameters vehicleParams, 
    //     BsplineGeometry geo, 
    //     PathKinematics path
    // )
    // {

    //     this.targetPointState = TPstate;
    //     this.vehicleRobotState = robot;
    //     this.vehicleRobotParms = vehicleParams;
    //     this.bsplineGeometry = geo;
    //     this.pathKinematics = path;    
    // }

    public void Initialize(
        TargetPointState TPstate,
        VehicleRobotState robot, 
        VehicleParameters vehicleParams, 
        BsplineGeometry geo, 
        PathKinematics path
    )
    {

        this.targetPointState = TPstate;
        this.vehicleRobotState = robot;
        this.vehicleRobotParms = vehicleParams;
        this.bsplineGeometry = geo;
        this.pathKinematics = path;    

        L1 = vehicleRobotParms.GetL1();
        L2 = vehicleRobotParms.GetL2();
        L3 = vehicleRobotParms.GetL3();

        Debug.Log("VehicleKinematics Initialize 完了");
    }

    public void CalculationVehicleKinematics()
    {

        using(VehicleKinematicsMarker.Auto())
        {
            ComputeUtilitiesForControlInput();

            CalculateLie();

            ComputeControlInputsWForVariable();

            // Debug.Log($"w1:{w1}, w2:{w2}, w3:{w3}");

            ComputeControlInputsU();

            // Debug.Log($"u1:{u1}, u2:{u2}, u3:{u3}");

        }


        // Debug.Log($"u1:{u1}, u2:{u2}, u3:{u3}, u4:{u4}");
    }

    public void ComputeUtilitiesForControlInput()
    {
        _cs1 = pathKinematics.GetCs1();// 先頭車両追従に関与
        _cs2 = pathKinematics.GetCs2();

        _d1c1ds11 = pathKinematics.GetD1c1ds11();// 先頭車両追従に関与
        _d2c1ds12 = pathKinematics.GetD2c1ds12();// 先頭車両追従に関与

        _d1c2ds21 = pathKinematics.GetD1c2ds21();
        _d2c2ds22 = pathKinematics.GetD2c2ds22();

        _phi1 = vehicleRobotState.GetPhi1();// 先頭車両追従に関与
        _phi2 = vehicleRobotState.GetPhi2();

        
        CalculatethetaP1(); // 先頭車両追従に関与
        CalculatethetaP2();
        CalculatethetaP3();

        _thetaP2d = pathKinematics.GetThetaP2d();

        CalculateD1();// 先頭車両追従に関与

        // Debug.Log($"_phi1:{_phi1}, _phi2:{_phi2}, thetaP1:{thetaP1}, thetaP2:{thetaP2}, thetaP3:{thetaP3}, _d1:{_d1}");

        // Power(c(s1(t)),2):
        _squaredCs1 = Mathf.Pow(_cs1, 2);// 先頭車両追従に関与
        // 1 - c(s1(t))*d1(t):
        _formulaOf1MinusCs1MulD1 = 1 - _cs1*_d1;// 先頭車両追従に関与
        // Power(1 - c(s1(t))*d1(t),2):
        _squaredFormulaOf1MinusCs1MulD1 = Mathf.Pow(_formulaOf1MinusCs1MulD1,2);
        // Power(1 - c(s1(t))*d1(t),3):
        _cubedFormulaOf1MinusCs1MulD1 = Mathf.Pow(_formulaOf1MinusCs1MulD1,3);

        // Power(Derivative(1)(c)(s1(t)),2):
        _squaredD1c1ds11 = Mathf.Pow(_d1c1ds11, 2);

        _cosThetaP1 = Mathf.Cos(thetaP1);
        _sinThetaP1 = Mathf.Sin(thetaP1);
        _tanThetaP1 = Mathf.Tan(thetaP1);
        _secThetaP1 = 1f / Mathf.Cos(thetaP1); // secThetaP1

        _squaredSecThetaP1 = Mathf.Pow(_secThetaP1, 2);
        _cubedSecThetaP1 = Mathf.Pow(_secThetaP1, 3);
        _squaredTanThetaP1 = Mathf.Pow(_tanThetaP1, 2);


        _secPhi1 = 1f / Mathf.Cos(_phi1);
        _tanPhi1 = Mathf.Tan(_phi1);
        _squaredSecPhi1 = Mathf.Pow(_secPhi1,2);

        _tanPhi2 = Mathf.Tan(_phi2);
        _secPhi2 = 1f / Mathf.Cos(_phi2);
        _squaredSecPhi2 = Mathf.Pow(_secPhi2,2);

        _cosThetaP2MinusThetaP3 = Mathf.Cos(thetaP2 - thetaP3);
        _secThetaP2MinusThetaP3 = 1f / Mathf.Cos(thetaP2 - thetaP3);
        _tanThetaP2MinusThetaP3 = Mathf.Tan(thetaP2 - thetaP3);

        _squaredSecThetaP2MinusThetaP3 = Mathf.Pow(_secThetaP2MinusThetaP3,2);
        _cubedSecThetaP2MinusThetaP3 = Mathf.Pow(_secThetaP2MinusThetaP3,3);
        _squaredTanThetaP2MinusThetaP3 = Mathf.Pow(_tanThetaP2MinusThetaP3,2);

        _cosThetaP1MinusThetaP3 = Mathf.Cos(thetaP1 - thetaP3);
        _sinThetaP1MinusThetaP3 = Mathf.Sin(thetaP1 - thetaP3);
        _squaredSinThetaP1MinusThetaP3 = Mathf.Pow(_sinThetaP1MinusThetaP3,2);

        _cosThetaP1MinusThetaP2 = Mathf.Cos(thetaP1 - thetaP2);
        _sinThetaP1MinusThetaP2 = Mathf.Sin(thetaP1 - thetaP2);

        _formulaOfCs1MulSecThetaP1 = _cs1*_secThetaP1;
        _formulaOfCs1MulCosThetaP1 = _cs1*_cosThetaP1;

        _squaredL2 = Mathf.Pow(L2,2);
        _squaredD1 = Mathf.Pow(_d1,2);;

        // Sec(thetaP2(t) - thetaP3(t)) * Sin(thetaP1(t) - thetaP3(t)):
        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3 = _secThetaP2MinusThetaP3*_sinThetaP1MinusThetaP3;
        // Cos(thetaP1(t) - thetaP3(t))*Sec(thetaP2(t) - thetaP3(t)):
        _formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3 = _cosThetaP1MinusThetaP3*_secThetaP2MinusThetaP3;
        // Cos(thetaP1(t) - thetaP2(t))*Sec(thetaP2(t) - thetaP3(t)):
        _formulaOfCosThetaP1MinusThetaP2MulSecThetaP2MinusThetaP3 = _cosThetaP1MinusThetaP2*_secThetaP2MinusThetaP3;
        // (c(s1(t))*Cos(thetaP1(t)))/(1 - c(s1(t))*d1(t)):
        _formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1 = _cs1*_cosThetaP1/_formulaOf1MinusCs1MulD1;
        // ((c(s1(t))*Cos(thetaP1(t))*d1(t)*Derivative(1)(c)(s1(t)))/Power(1 - c(s1(t))*d1(t),2)):
        _formulaOfCs1MulCosThetaP1MulD1MulD1c1ds11DevSquaredFormulaOf1MinusCs1MulD1 = _cs1*_cosThetaP1*_d1*_d1c1ds11/_squaredFormulaOf1MinusCs1MulD1;
        // (Sec(thetaP2(t) - thetaP3(t))*Sin(thetaP1(t) - thetaP3(t)))/L2:
        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2 = _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3/L2;
        // (c(s1(t))*Sin(thetaP1(t)))/(1 - c(s1(t))*d1(t)):
        _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1 = _cs1*_sinThetaP1 / _formulaOf1MinusCs1MulD1;
        // (Cos(thetaP1(t))*Derivative(1)(c)(s1(t)))/(1 - c(s1(t))*d1(t)):
        _formulaOfFormulaOfCosThetaP1MulD1c1ds11DevFormulaOf1MinusCs1MulD1 = _cosThetaP1*_d1c1ds11/_formulaOf1MinusCs1MulD1;
        // Cos(thetaP1(t))*Derivative(1)(c)(s1(t)):
        _formulaOfCosThetaP1MulD1c1ds11 = _cosThetaP1*_d1c1ds11;
        // Power(1 - c(s1(t))*d1(t),2)*Power(Sec(thetaP1(t)),2):
        _formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1 = _squaredFormulaOf1MinusCs1MulD1*_squaredSecThetaP1;
        // d1(t)*Sec(thetaP1(t)):
        _formulaOfD1MulSecThetaP1 = _d1*_secThetaP1;
        // Tan(phi1(t))/L1:
        _formulaTanPhi1DevL1 = _tanPhi1 / L1;
        // (1 - c(s1(t))*d1(t))*Sec(thetaP1(t)):
        _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1 = _formulaOf1MinusCs1MulD1*_secThetaP1;
        // Power(Sec(thetaP2(t) - thetaP3(t)),3)*Sin(thetaP1(t) - thetaP3(t)):
        _formulaOfCubedSecThetaP2MinusThetaP3MulSinThetaP1MinusThetaP3 = _cubedSecThetaP2MinusThetaP3*_sinThetaP1MinusThetaP3;
    }

    // 先頭車両追従に関与
    public void CalculateD1()
    {
        float _d1;

        float _x_curr = vehicleRobotState.GetX1();
        float _y_curr = vehicleRobotState.GetY1();
        float _rx1 = bsplineGeometry.GetRx1();
        float _ry1 = bsplineGeometry.GetRy1();

        float _d1rx1dq11 = bsplineGeometry.GetD1Rx1du11();
        float _d1ry1dq11 = bsplineGeometry.GetD1Ry1du11();

        float _norm1 = Mathf.Sqrt(_d1rx1dq11*_d1rx1dq11 + _d1ry1dq11*_d1ry1dq11);

        Vector2 _e;
        _e.x = _d1rx1dq11 / _norm1;
        _e.y = _d1ry1dq11 / _norm1;

        Vector2 _n = new Vector2(-_e.y, _e.x); // 法線ベクトル
        
        float _dx = _x_curr - _rx1;
        float _dy = _y_curr - _ry1;

        _d1 = _dx*_n.x + _dy*_n.y;

        // 第一操作点からベジェ曲線へ下ろした時に垂直となる接戦の角度
        SetD1(_d1);
    }

    // 先頭車両追従に関与
    public void CalculatethetaP1()
    {
        // Debug.Log($"vehicleRobotState.GetTheta1():{vehicleRobotState.GetTheta1()}, pathKinematics.GetThetaT1():{pathKinematics.GetThetaT1()}");
        SetThetaP1(NormalizeAngle(vehicleRobotState.GetTheta1() - pathKinematics.GetThetaT1()));
    }

    public void CalculatethetaP2()
    {
        SetThetaP2(NormalizeAngle(vehicleRobotState.GetTheta2() - pathKinematics.GetThetaT1()));
    }

    public void CalculatethetaP3()
    {
        SetThetaP3(NormalizeAngle(vehicleRobotState.GetTheta3() - pathKinematics.GetThetaT1()));
    }

    private float NormalizeAngle(float angle)
    {
        return Mathf.Atan2(Mathf.Sin(angle), Mathf.Cos(angle));
    }

    private void CalculateLie()
    {

        //z22
        L1f1h2 = _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                    _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2);

        //z21
        L2f1h2 = (_formulaOf1MinusCs1MulD1)*(-(_squaredCs1/(_formulaOf1MinusCs1MulD1)) - 
                    _formulaOfCs1MulSecThetaP1*(-((_formulaOfCs1MulCosThetaP1)/
                        (_formulaOf1MinusCs1MulD1)) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2))*
                _tanThetaP1 + _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + _formulaTanPhi1DevL1)*
                (_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    ((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2 + 
                        _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1) + 
                    _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                    _tanThetaP1) + (_squaredFormulaOf1MinusCs1MulD1*
                    _squaredSecThetaP1*_secThetaP2MinusThetaP3*
                    _sinThetaP1MinusThetaP3*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                    _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                    _tanThetaP2MinusThetaP3)/L2 + 
                _formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) - 
                    (_formulaOfCosThetaP1MinusThetaP2MulSecThetaP2MinusThetaP3*
                        _tanPhi2)/L3)*(-((_cosThetaP1MinusThetaP3*
                        _secThetaP2MinusThetaP3)/L2) - 
                    (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _tanThetaP2MinusThetaP3)/L2) - 
                _formulaOfD1MulSecThetaP1*(-((_formulaOfCs1MulCosThetaP1)/
                        (_formulaOf1MinusCs1MulD1)) + 
                    _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                _d1c1ds11 + _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                (-((_formulaOfCs1MulCosThetaP1*_d1*_d1c1ds11)/
                        _squaredFormulaOf1MinusCs1MulD1) - 
                    _formulaOfFormulaOfCosThetaP1MulD1c1ds11DevFormulaOf1MinusCs1MulD1);

        L3f1h2 = -(_d1*(-(_squaredCs1/(_formulaOf1MinusCs1MulD1)) - 
                    _formulaOfCs1MulSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2))*
                    _tanThetaP1*_d1c1ds11) - 
                _formulaOfD1MulSecThetaP1*(-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + _formulaTanPhi1DevL1)*
                (_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    ((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2 + 
                        _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1) + 
                    _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                    _tanThetaP1)*_d1c1ds11 - 
                (2f*_d1*(_formulaOf1MinusCs1MulD1)*_squaredSecThetaP1*
                    _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                    _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                    _tanThetaP2MinusThetaP3*_d1c1ds11)/L2 - 
                2f*_d1*(_formulaOf1MinusCs1MulD1)*_squaredSecThetaP1*
                (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) - 
                    (_formulaOfCosThetaP1MinusThetaP2MulSecThetaP2MinusThetaP3*_tanPhi2)/L3)*(-((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2) - 
                    (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _tanThetaP2MinusThetaP3)/L2)*_d1c1ds11 + 
                _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                (_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    ((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2 + 
                        _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1) + 
                    _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                    _tanThetaP1)*(-_formulaOfCs1MulCosThetaP1MulD1MulD1c1ds11DevSquaredFormulaOf1MinusCs1MulD1 - 
                    _formulaOfFormulaOfCosThetaP1MulD1c1ds11DevFormulaOf1MinusCs1MulD1) + 
                (_formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                    _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                    _tanThetaP2MinusThetaP3*
                    (-_formulaOfCs1MulCosThetaP1MulD1MulD1c1ds11DevSquaredFormulaOf1MinusCs1MulD1 - 
                    _formulaOfFormulaOfCosThetaP1MulD1c1ds11DevFormulaOf1MinusCs1MulD1))/L2 + 
                _formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                (-((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2) - 
                    (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _tanThetaP2MinusThetaP3)/L2)*
                (-_formulaOfCs1MulCosThetaP1MulD1MulD1c1ds11DevSquaredFormulaOf1MinusCs1MulD1 - 
                    _formulaOfFormulaOfCosThetaP1MulD1c1ds11DevFormulaOf1MinusCs1MulD1) - 
                2f*_formulaOfD1MulSecThetaP1*_d1c1ds11*
                (-_formulaOfCs1MulCosThetaP1MulD1MulD1c1ds11DevSquaredFormulaOf1MinusCs1MulD1 - 
                    _formulaOfFormulaOfCosThetaP1MulD1c1ds11DevFormulaOf1MinusCs1MulD1) + 
                _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                    _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                ((_formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                        _formulaOfCubedSecThetaP2MinusThetaP3MulSinThetaP1MinusThetaP3*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2))/
                    L2 - (_cs1*_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _tanThetaP1*_tanThetaP2MinusThetaP3)/L2 + 
                    (_formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                        _squaredSecThetaP2MinusThetaP3*
                        _squaredSinThetaP1MinusThetaP3*
                        _squaredTanThetaP2MinusThetaP3)/_squaredL2 + 
                    (_formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                        _squaredTanThetaP2MinusThetaP3)/L2 + 
                    _formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                    (-((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/
                        L2) - (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*_tanThetaP2MinusThetaP3)/L2)*
                    (-((_secThetaP2MinusThetaP3*_sinThetaP1MinusThetaP2*
                            _tanPhi2)/L3) - 
                        (_formulaOfCosThetaP1MinusThetaP2MulSecThetaP2MinusThetaP3*_tanPhi2*_tanThetaP2MinusThetaP3)/L3) + 
                    _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaTanPhi1DevL1)*((_cosThetaP1MinusThetaP3*
                        _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        _secThetaP2MinusThetaP3*_tanThetaP2MinusThetaP3)/L2 + 
                        (_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _tanThetaP1*_tanThetaP2MinusThetaP3)/L2) + 
                    _formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) - 
                        (_formulaOfCosThetaP1MinusThetaP2MulSecThetaP2MinusThetaP3*_tanPhi2)/L3)*(-((_formulaOfCubedSecThetaP2MinusThetaP3MulSinThetaP1MinusThetaP3)/L2) - 
                        (_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3*
                        _tanThetaP2MinusThetaP3)/L2 - 
                        (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _squaredTanThetaP2MinusThetaP3)/L2) - 
                    (_formulaOfD1MulSecThetaP1*_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*_tanThetaP2MinusThetaP3*
                        _d1c1ds11)/L2) + 
                _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) - 
                    (_formulaOfCosThetaP1MinusThetaP2MulSecThetaP2MinusThetaP3*_tanPhi2)/L3)*(-((_squaredFormulaOf1MinusCs1MulD1*
                        _squaredSecThetaP1*_formulaOfCubedSecThetaP2MinusThetaP3MulSinThetaP1MinusThetaP3*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                            _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2 
                ))/L2) - (_cosThetaP1MinusThetaP3*_squaredFormulaOf1MinusCs1MulD1*
                        _squaredSecThetaP1*_secThetaP2MinusThetaP3*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                        _tanThetaP2MinusThetaP3)/L2 - 
                    (_formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                        _squaredTanThetaP2MinusThetaP3)/L2 - 
                    _cs1*_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*_tanThetaP1*
                    (-((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2) - 
                        (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _tanThetaP2MinusThetaP3)/L2) + 
                    (_formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _tanThetaP2MinusThetaP3*
                        (-((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/
                            L2) - (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*_tanThetaP2MinusThetaP3)/L2 
                ))/L2 + (_cosThetaP1MinusThetaP2*_squaredFormulaOf1MinusCs1MulD1*
                        _squaredSecThetaP1*_secThetaP2MinusThetaP3*_tanPhi2*_tanThetaP2MinusThetaP3*
                        (-((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/
                            L2) - (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*_tanThetaP2MinusThetaP3)/L2 
                ))/L3 + _formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) - 
                        (_formulaOfCosThetaP1MinusThetaP2MulSecThetaP2MinusThetaP3*_tanPhi2)/L3)*(-(_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2) + 
                        (_formulaOfCubedSecThetaP2MinusThetaP3MulSinThetaP1MinusThetaP3)/L2 + 
                        (2f*_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3*
                        _tanThetaP2MinusThetaP3)/L2 + 
                        (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _squaredTanThetaP2MinusThetaP3)/L2) + 
                    _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaTanPhi1DevL1)*(_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        ((_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3)/
                            L2 - (_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3*_tanThetaP2MinusThetaP3)/
                            L2) + _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*_tanThetaP1*
                        (-((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/
                            L2) - (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*_tanThetaP2MinusThetaP3)/L2 
                )) - _formulaOfD1MulSecThetaP1*(-((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2) - 
                        (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _tanThetaP2MinusThetaP3)/L2)*_d1c1ds11) + 
                (_formulaOf1MinusCs1MulD1)*_tanThetaP1*
                (-((_squaredCs1*_d1*_d1c1ds11)/
                        _squaredFormulaOf1MinusCs1MulD1) - 
                    (2f*_cs1*_d1c1ds11)/(_formulaOf1MinusCs1MulD1) - 
                    _secThetaP1*(-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                    _d1c1ds11 - 
                    _formulaOfCs1MulSecThetaP1*(-_formulaOfCs1MulCosThetaP1MulD1MulD1c1ds11DevSquaredFormulaOf1MinusCs1MulD1 - 
                        _formulaOfFormulaOfCosThetaP1MulD1c1ds11DevFormulaOf1MinusCs1MulD1)) + 
                (_formulaOf1MinusCs1MulD1)*_tanThetaP1*
                (-(_cs1*(-(_squaredCs1/(_formulaOf1MinusCs1MulD1)) - 
                        _formulaOfCs1MulSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                            (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3)/
                            L2))*_tanThetaP1) + 
                    _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaTanPhi1DevL1)*(-(_formulaOfCs1MulSecThetaP1*
                        ((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/
                            L2 + _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1)) - 
                        _formulaOfCs1MulSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                        _tanThetaP1) - (_squaredCs1*
                        (_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        ((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/
                            L2 + _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                            (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3)/
                            L2)*_tanThetaP1))/(_formulaOf1MinusCs1MulD1) - 
                    _formulaOfCs1MulSecThetaP1*(-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + _formulaTanPhi1DevL1)*
                    (_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        ((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/
                            L2 + _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                        _tanThetaP1) - (_squaredCs1*_secThetaP1*
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _tanThetaP2MinusThetaP3)/L2 - 
                    (2f*_cs1*(_formulaOf1MinusCs1MulD1)*_squaredSecThetaP1*
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                        _tanThetaP2MinusThetaP3)/L2 - 
                    _squaredCs1*_secThetaP1*
                    (-((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2) - 
                        (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _tanThetaP2MinusThetaP3)/L2) - 
                    2f*_cs1*(_formulaOf1MinusCs1MulD1)*_squaredSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) - 
                        (_formulaOfCosThetaP1MinusThetaP2MulSecThetaP2MinusThetaP3*_tanPhi2)/L3)*(-((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2) - 
                        (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _tanThetaP2MinusThetaP3)/L2) + 
                    (_squaredCs1*_d1*_d1c1ds11)/
                    _squaredFormulaOf1MinusCs1MulD1 - 
                    _secThetaP1*(-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                    _d1c1ds11 + 
                    _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    ((-2f*_squaredCs1*_cosThetaP1*_d1*
                        _d1c1ds11)/_cubedFormulaOf1MinusCs1MulD1 - 
                        (2f*_cs1*_formulaOfCosThetaP1MulD1c1ds11)/
                        _squaredFormulaOf1MinusCs1MulD1) - 
                    _formulaOfCs1MulSecThetaP1*(-_formulaOfCs1MulCosThetaP1MulD1MulD1c1ds11DevSquaredFormulaOf1MinusCs1MulD1 - 
                        _formulaOfFormulaOfCosThetaP1MulD1c1ds11DevFormulaOf1MinusCs1MulD1)) + 
                _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + _formulaTanPhi1DevL1)*
                (-(_formulaOfD1MulSecThetaP1*((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2 + 
                        _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1)*
                        _d1c1ds11) - 
                    _formulaOfD1MulSecThetaP1*(-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                    _tanThetaP1*_d1c1ds11 + 
                    _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*_tanThetaP1*
                    (-_formulaOfCs1MulCosThetaP1MulD1MulD1c1ds11DevSquaredFormulaOf1MinusCs1MulD1 - 
                        _formulaOfFormulaOfCosThetaP1MulD1c1ds11DevFormulaOf1MinusCs1MulD1) + 
                    _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    ((_cs1*_d1*_sinThetaP1*_d1c1ds11)/
                        _squaredFormulaOf1MinusCs1MulD1 + 
                        (_sinThetaP1*_d1c1ds11)/(_formulaOf1MinusCs1MulD1))) + 
                _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + _formulaTanPhi1DevL1)*
                ((_formulaOf1MinusCs1MulD1)*_squaredSecThetaP1*
                    (-(_squaredCs1/(_formulaOf1MinusCs1MulD1)) - 
                        _formulaOfCs1MulSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2))  
                + (_formulaOf1MinusCs1MulD1)*_tanThetaP1*
                    (-(_formulaOfCs1MulSecThetaP1*
                        ((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/
                            L2 + _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1)) - 
                        _formulaOfCs1MulSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                        _tanThetaP1) + _cs1*_tanThetaP1*
                    (_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        ((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/
                            L2 + _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                        _tanThetaP1) + _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaTanPhi1DevL1)*_tanThetaP1*
                    (_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        ((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/
                            L2 + _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                        _tanThetaP1) + _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaTanPhi1DevL1)*(_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        (_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1 - 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)  
                + (_formulaOf1MinusCs1MulD1)*_cubedSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)  
                + 2f*_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        ((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/
                            L2 + _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1)*
                        _tanThetaP1 + _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                        _squaredTanThetaP1) + 
                    (_formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                        _secThetaP2MinusThetaP3*
                        ((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/
                        L2 + _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1)*
                        _sinThetaP1MinusThetaP3*_tanThetaP2MinusThetaP3)/L2 + 
                    (_cosThetaP1MinusThetaP3*_squaredFormulaOf1MinusCs1MulD1*
                        _squaredSecThetaP1*_secThetaP2MinusThetaP3*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                        _tanThetaP2MinusThetaP3)/L2 + 
                    (2f*_formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                        _tanThetaP1*_tanThetaP2MinusThetaP3)/L2 + 
                    _formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) - 
                        (_formulaOfCosThetaP1MinusThetaP2MulSecThetaP2MinusThetaP3*_tanPhi2)/L3)*(_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2 - 
                        (_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3*
                        _tanThetaP2MinusThetaP3)/L2) + 
                    _formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                    (_formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1 + 
                        (_secThetaP2MinusThetaP3*_sinThetaP1MinusThetaP2*
                        _tanPhi2)/L3)*(-((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2) - 
                        (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _tanThetaP2MinusThetaP3)/L2) + 
                    2f*_formulaOfSquaredFormulaOf1MinusCs1MulD1MulSquaredSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) - 
                        (_formulaOfCosThetaP1MinusThetaP2MulSecThetaP2MinusThetaP3*_tanPhi2)/L3)*_tanThetaP1*
                    (-((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2) - 
                        (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                        _tanThetaP2MinusThetaP3)/L2) - 
                    _formulaOfD1MulSecThetaP1*((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2 + 
                        _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1)*
                    _d1c1ds11 - 
                    _formulaOfD1MulSecThetaP1*(-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                    _tanThetaP1*_d1c1ds11 + 
                    _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*_tanThetaP1*
                    (-_formulaOfCs1MulCosThetaP1MulD1MulD1c1ds11DevSquaredFormulaOf1MinusCs1MulD1 - 
                        _formulaOfFormulaOfCosThetaP1MulD1c1ds11DevFormulaOf1MinusCs1MulD1) + 
                    _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    ((_cs1*_d1*_sinThetaP1*_d1c1ds11)/
                        _squaredFormulaOf1MinusCs1MulD1 + 
                        (_sinThetaP1*_d1c1ds11)/(_formulaOf1MinusCs1MulD1))) - 
                _formulaOfD1MulSecThetaP1*(-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                    _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                _d2c1ds12 + _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                ((-2f*_formulaOfCs1MulCosThetaP1*_squaredD1*
                        _squaredD1c1ds11)/_cubedFormulaOf1MinusCs1MulD1 - 
                    (2f*_cosThetaP1*_d1*_squaredD1c1ds11)/
                    _squaredFormulaOf1MinusCs1MulD1 - 
                    (_formulaOfCs1MulCosThetaP1*_d1*_d2c1ds12)/
                    _squaredFormulaOf1MinusCs1MulD1 - 
                    (_cosThetaP1*_d2c1ds12)/(_formulaOf1MinusCs1MulD1));



        // z32
        L1f1h3 = (_formulaOf1MinusCs1MulD1)*_tanThetaP1;// Lie微分


        // z31
        L2f1h3 = _squaredFormulaOf1MinusCs1MulD1*_cubedSecThetaP1*
                (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + _formulaTanPhi1DevL1) - 
                _cs1*(_formulaOf1MinusCs1MulD1)*_squaredTanThetaP1 - 
                _d1*_tanThetaP1*_d1c1ds11;


        // 先頭車両追従に関与
        L3f1h3 = -2f*_d1*(_formulaOf1MinusCs1MulD1)*_cubedSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + _formulaTanPhi1DevL1)*
                    _d1c1ds11 + _cs1*_d1*_squaredTanThetaP1*
                    _d1c1ds11 - (_formulaOf1MinusCs1MulD1)*_squaredTanThetaP1*
                    _d1c1ds11 + _squaredFormulaOf1MinusCs1MulD1*
                    _cubedSecThetaP1*(-((_formulaOfCs1MulCosThetaP1*_d1*
                            _d1c1ds11)/_squaredFormulaOf1MinusCs1MulD1) - 
                        _formulaOfFormulaOfCosThetaP1MulD1c1ds11DevFormulaOf1MinusCs1MulD1) + 
                    _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                    (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + _formulaTanPhi1DevL1)*
                    (-(_cs1*(_formulaOf1MinusCs1MulD1)*_squaredSecThetaP1*
                            _tanThetaP1) + 3*_squaredFormulaOf1MinusCs1MulD1*
                        _cubedSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                            _formulaTanPhi1DevL1)*_tanThetaP1 - 
                        _d1*_squaredSecThetaP1*_d1c1ds11) + 
                    (_formulaOf1MinusCs1MulD1)*_tanThetaP1*
                    (-(_squaredCs1*_squaredSecThetaP1) - 
                        2f*_cs1*(_formulaOf1MinusCs1MulD1)*_cubedSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                            _formulaTanPhi1DevL1) + _squaredCs1*_squaredTanThetaP1 - 
                        _tanThetaP1*_d1c1ds11) - 
                    _d1*_tanThetaP1*_d2c1ds12;

        Lf2L2f1h2 = -((_cosThetaP1MinusThetaP2*_squaredFormulaOf1MinusCs1MulD1*
                        _squaredSecPhi2*_squaredSecThetaP1*
                        _secThetaP2MinusThetaP3*
                        (-((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2) - 
                            (_formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3*
                            _tanThetaP2MinusThetaP3)/L2))/L3);

        Lf3L2f1h2 = ((_formulaOf1MinusCs1MulD1)*_squaredSecPhi1*_secThetaP1*
                        (_formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        ((_formulaOfCosThetaP1MinusThetaP3MulSecThetaP2MinusThetaP3)/L2 + 
                            _formulaOfCs1MulSinThetaP1DevFormulaOf1MinusCs1MulD1) + 
                        _formulaOfFormulaOf1MinusCs1MulD1MulFormulaOfSecThetaP1*
                        (-(_formulaOfFormulaOfCs1MulCosThetaP1DevFormulaOf1MinusCs1MulD1) + 
                            _formulaOfFormulaSecThetaP2MinusThetaP3MulFormulaSinThetaP1MinusThetaP3DevL2)*
                        _tanThetaP1))/L1;

        // 先頭車両追従に関与
        Lf3L2f1h3 = (_squaredFormulaOf1MinusCs1MulD1*_squaredSecPhi1*_cubedSecThetaP1)/L1;
    }

        //可変速
    public void ComputeControlInputsWForVariable()
    {
        //　制御入力　w１
        // ok
        z22 = L1f1h2;
        z21 = L2f1h2;

        float _d1thetaP2dds11 = pathKinematics.GetD1thetaP2dds11();
        float _d2thetaP2dds12 = pathKinematics.GetD2thetaP2dds12();
        float _d3thetaP2dds13 = pathKinematics.GetD3thetaP2dds13();

        // Debug.Log($"_d1thetaP2dds11:{_d1thetaP2dds11}, _d2thetaP2dds12:{_d2thetaP2dds12}, _d3thetaP2dds13:{_d3thetaP2dds13}");


        // 車両の制御入力w1, w3を目標点のv1, v2に対応
        w1 = targetPointState.getV1();
        // w3 = targetPointState.getV2();

        // 符号付き微分
        _thetaP2d_1 = Mathf.Sign(w1) * _d1thetaP2dds11;
        _thetaP2d_2 = _d2thetaP2dds12;
        _thetaP2d_3 = Mathf.Sign(w1) * _d3thetaP2dds13;

        // 
        _thetaP2_1 = Mathf.Sign(w1) * z22;
        _thetaP2_2 = z21;
        _thetaP2_3 = (_thetaP2d_3 + k1*(_thetaP2d_2 - _thetaP2_2) + k2*(_thetaP2d_2 + k1*(_thetaP2d_1 - _thetaP2_1) - _thetaP2_2)) +
                k3*(_thetaP2d_2 + k1*(_thetaP2d_1 - _thetaP2_1) + k2*(_thetaP2d_1 + k1*(_thetaP2d - thetaP2) - _thetaP2_1) - _thetaP2_2);


        w2 = Mathf.Sign(w1)*_thetaP2_3*w1;

        z33 = _d1;
        z32 = L1f1h3;
        z31 = L2f1h3;

        w3 = p31*Mathf.Abs(w1)*z31 + p32*w1*z32 + p33*Mathf.Abs(w1)*z33;

    }

    public void ComputeControlInputsU()
    {
        float tildaU1 = w1; // 先頭車両追従に関与
        float tildaU3 = (w3 - L3f1h3*tildaU1) / Lf3L2f1h3; // 先頭車両追従に関与
        float tildaU2 = (w2 - (L3f1h2*tildaU1 + Lf3L2f1h2*tildaU3)) / Lf2L2f1h2;

        u1 = (_formulaOf1MinusCs1MulD1)/_cosThetaP1*tildaU1; // 先頭車両追従に関与
        u2 = tildaU3; // 先頭車両追従に関与
        u3 = tildaU2; 
        u4 = _cosThetaP1MinusThetaP2 / _cosThetaP2MinusThetaP3 * u1 ;

        // Debug.Log($"w1:{w1}, w2:{w2}, w3:{w3}, u1:{u1}, u2:{u2}, u3:{u3}");
        // Debug.Log($"tildaU1:{tildaU1}, tildaU3:{tildaU3}, tildaU2:{tildaU2}");

    }

    public void SetD1(float v) { _d1 = v; }
    public void SetThetaP1(float v) { thetaP1 = v; }
    public void SetThetaP2(float v) { thetaP2 = v; }
    public void SetThetaP3(float v) { thetaP3 = v; }

    public float GetU1() => u1;
    public float GetU2() => u2;
    public float GetU3() => u3;
    public float GetU4() => u4;
}


