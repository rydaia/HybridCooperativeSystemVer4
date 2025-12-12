using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Profiling;

public class PathKinematics : MonoBehaviour
{

    static ProfilerMarker pathKinematicsMarker =
        new ProfilerMarker("PathKinematics.CalculationPathKinematics");

    public BsplineGeometry bsplineGeometry;  // ← 外からセット


    // u1
    private float _rx1;
    private float _ry1;

    private float _d1rx1du11, _d1ry1du11;
    private float _d2rx1du12, _d2ry1du12;
    private float _d3rx1du13, _d3ry1du13;
    private float _d4rx1du14, _d4ry1du14;

    public float cs1;
    public float d1c1ds11;
    public float d2c1ds12;

    public float cs2;
    public float d1c2ds21;
    public float d2c2ds22;

    // u2
    private float _rx2;
    private float _ry2;

    private float _d1rx2du21, _d1ry2du21;
    private float _d2rx2du22, _d2ry2du22;
    private float _d3rx2du23, _d3ry2du23;
    private float _d4rx2du24, _d4ry2du24;


    private float thetaT1;
    private float thetaT2;
    private float thetaP2d;

    // 微分
    public float d1u1ds11, d2u1ds12, d3u1ds13;
    public float d1u2ds21, d2u2ds22, d3u2ds23;
    public float d1u2ds11, d2u2ds12, d3u2ds13;
    public float d1thetaT1ds11, d2thetaT1ds12, d3thetaT1ds13;
    public float d1thetaT2ds21, d2thetaT2ds22, d3thetaT2ds23;
    public float d1s2ds11, d2s2ds12, d3s2ds13;
    public float d1thetaP2dds11, d2thetaP2dds12, d3thetaP2dds13;

    // u1
    private float _squaredRx1;
    private float _squaredRy1;

    private float _squaredD1rx1du11;
    private float _cubedD1rx1du11;
    private float _squaredD1ry1du11;
    private float _cubedD1ry1du11;

    private float _squaredD2rx1du12;
    private float _cubedD2rx1du12;
    private float _squaredD2ry1du12;
    private float _cubedD2ry1du12;

    private float _formulaOfD1ry1du11MulD2rx1du12;
    private float _formulaOfD1ry1du11MulD3rx1du13;
    private float _formulaOfD1ry1du11MulD3ry1du13;
    private float _formulaOfD1rx1du11MulD2ry1du12;
    private float _formulaOfD2rx1du12MulD2ry1du12;

    private float _formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11;

    // u2
    private float _squaredRx2;
    private float _squaredRy2;

    private float _squaredD1rx2du21;
    private float _cubedD1rx2du21;
    private float _squaredD1ry2du21;
    private float _cubedD1ry2du21;

    private float _squaredD2rx2du22;
    private float _cubedD2rx2du22;
    private float _squaredD2ry2du22;
    private float _cubedD2ry2du22;

    private float _formulaOfD1ry2du21MulD2rx2du22;
    private float _formulaOfD1ry2du21MulD3rx2du23;
    private float _formulaOfD1ry2du21MulD3ry2du23;
    private float _formulaOfD1rx2du21MulD2ry2du22;
    private float _formulaOfD2rx2du22MulD2ry2du22;

    private float _formulaOfSquaredD1rx2du21PlusSquaredD1ry2du21;

    public float _formulaOfRx1SubRx2;
    public float _formulaOfRy1SubRy2;
    public float _squaredFormulaOfRx1SubRx2;
    public float _squaredFormulaOfRy1SubRy2;

    // 

    private float _cosThetaP2d;
    private float _sinThetaP2d;

    private float _formulaOfThetaP2dPlusThetaT1MinusThetaT2;
    private float _secThetaP2dPlusThetaT1MinusThetaT2;
    private float _tanThetaP2dPlusThetaT1MinusThetaT2;


    public void Initialize(TrajectoryGenerator trajectoryGenerator, BsplineGeometry bsplineGeometry)
    {
        this.bsplineGeometry = bsplineGeometry;

        int N = trajectoryGenerator.GetN();

        Debug.Log("PathKinematics Initialize 完了");
    }
    public void CalculationPathKinematics()
    {
    
        using (pathKinematicsMarker.Auto())
        {
            
            ComputeUtilitiesForBezierGeometry();

            CalculateThetaT1(); //ok
            CalculateThetaT2();
            CalculateThetaP2d();

            ComputeUtilitiesForPathKinematics();

            // 曲率計算 u1
            CalculateCurvatureOfU1(); //ok
            CalculateCurvatureOfU1Derivative1(); // ok
            CalculateCurvatureOfU1Derivative2();
            // 曲率計算 u2
            CalculateCurvatureOfU2(); //ok
            CalculateCurvatureOfU2Derivative1(); // ok
            CalculateCurvatureOfU2Derivative2();

            // u1の一階s1微分
            CalculateU1DerivativeOfS11();
            // u2の一階s2微分
            CalculateU2DerivativeOfS21();
            // thetaT1の一階s1微分
            CalculateThetaT1DerivativeOfS11();
            // thetaT2の一階s2微分
            CalculateThetaT2DerivativeOfS21();
            // s2の1階s1微分
            CalculateS2DerivativeOfS11();
            // u2の一階s1微分
            CalculateU2DerivativeOfS11();
            // thetaP2dの1階s1微分
            CalculateThetaP2dDerivativeOfS11(); 

            // u1の2階s1微分
            CalculateU1DerivativeOfS12();
            // u2の2階s2微分
            CalculateU2DerivativeOfS22();
            // thetaT1の2階s1微分
            CalculateThetaT1DerivativeOfS12();
            // thetaT2の2階s2微分
            CalculateThetaT2DerivativeOfS22();
            // s2の2階s1微分
            CalculateS2DerivativeOfS12();
            // u2の2階s1微分
            CalculateU2DerivativeOfS12();
            // thetaP2dの2階s1微分
            CalculateThetaP2dDerivativeOfS12(); 

            // u1の３階s1微分
            CalculateU1DerivativeOfS13();
            // u2の3階s2微分
            CalculateU2DerivativeOfS23();
            // thetaT1の３階s1微分
            CalculateThetaT1DerivativeOfS13();
            // thetaT2の3階s2微分
            CalculateThetaT2DerivativeOfS23();
            // s2の3階s1微分
            CalculateS2DerivativeOfS13();
            // u2の３階s1微分
            CalculateU2DerivativeOfS13();
            // thetaP2dの３階s1微分
            CalculateThetaP2dDerivativeOfS13(); 

            // Debug.Log($"d1thetaP2dds11:{d1thetaP2dds11}, d2thetaP2dds12:{d2thetaP2dds12}, d3thetaP2dds13:{d3thetaP2dds13}");

        }
    }


    // 曲率計算 ok
    public void CalculateCurvatureOfU1()
    {
        float _cs1;

        _cs1 = -((_squaredD1ry1du11*(_formulaOfD1ry1du11MulD2rx1du12 - _formulaOfD1rx1du11MulD2ry1du12)) / Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 2.5f))
        + (_squaredD1rx1du11*(-(_formulaOfD1ry1du11MulD2rx1du12) + _formulaOfD1rx1du11MulD2ry1du12)) / Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 2.5f);

        SetCs1(_cs1);
    }
    
    // 曲率1次微分 ok
    public void CalculateCurvatureOfU1Derivative1()
    {
        float _d1c1ds11;

        _d1c1ds11 = (_d1rx1du11*((_d1ry1du11*Mathf.Pow(-(_formulaOfD1ry1du11MulD2rx1du12) + _formulaOfD1rx1du11MulD2ry1du12,2)) /
                Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 3.5f) + (-(_cubedD1ry1du11*_squaredD2rx1du12) + _d1rx1du11*_squaredD1ry1du11 *
                    (5.0f*_formulaOfD2rx1du12MulD2ry1du12 - _formulaOfD1ry1du11MulD3rx1du13) - _cubedD1rx1du11*(3.0f*_formulaOfD2rx1du12MulD2ry1du12 + _formulaOfD1ry1du11MulD3rx1du13) +
                    Mathf.Pow(_d1rx1du11, 4)*_d3ry1du13 + _squaredD1rx1du11*_d1ry1du11*(3.0f*_squaredD2rx1du12 - 4.0f*_squaredD2ry1du12 + _formulaOfD1ry1du11MulD3ry1du13)) /
                Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 3.5f))) / Mathf.Sqrt(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11) -
                (_d1ry1du11*((_d1rx1du11 *Mathf.Pow(-(_formulaOfD1ry1du11MulD2rx1du12) +_formulaOfD1rx1du11MulD2ry1du12,2)) /
                    Mathf.Pow(_squaredD1rx1du11 +_squaredD1ry1du11, 3.5f) + (_cubedD1ry1du11*(-3*_formulaOfD2rx1du12MulD2ry1du12 + _formulaOfD1ry1du11MulD3rx1du13) +
                        _squaredD1rx1du11*_d1ry1du11*(5.0f*_formulaOfD2rx1du12MulD2ry1du12 + _formulaOfD1ry1du11MulD3rx1du13) -
                        _d1rx1du11*_squaredD1ry1du11*(4.0f*_squaredD2rx1du12 -
                            3.0f*_squaredD2ry1du12 + _formulaOfD1ry1du11MulD3ry1du13) - _cubedD1rx1du11*(_squaredD2ry1du12 + _formulaOfD1ry1du11MulD3ry1du13)) /
                    Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 3.5f))) / Mathf.Sqrt(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11);


        SetD1c1ds11(_d1c1ds11);
    }
    
    // 曲率2次微分 ok
    public void CalculateCurvatureOfU1Derivative2()
    {

        float _d2c1ds12;

        _d2c1ds12 = (-(Mathf.Pow(_d1rx1du11, 4) *
                (4.0f*_d2ry1du12*_d3rx1du13 +
                    6.0f*_d2rx1du12*_d3ry1du13 +
                    _d1ry1du11*_d4rx1du14)) +
                _squaredD1rx1du11*_d1ry1du11 *
                (-15.0f*_cubedD2rx1du12 +
                    _d2rx1du12*(39.0f*_squaredD2ry1du12 -
                        2.0f*_formulaOfD1ry1du11MulD3ry1du13) +
                    2.0f*_d1ry1du11*(_d2ry1du12*_d3rx1du13 -
                        _d1ry1du11*_d4rx1du14)) +
                _cubedD1ry1du11 *
                (3.0f*_cubedD2rx1du12 +
                    _d2rx1du12*(-15.0f*_squaredD2ry1du12 +
                        4.0f*_formulaOfD1ry1du11MulD3ry1du13) +
                    _d1ry1du11*(6.0f*_d2ry1du12*_d3rx1du13 -
                        _d1ry1du11*_d4rx1du14)) +
                Mathf.Pow(_d1rx1du11, 5)*_d4ry1du14 +
                _d1rx1du11*_squaredD1ry1du11 *
                (-39.0f*_squaredD2rx1du12*_d2ry1du12 +
                    15.0f*_cubedD2ry1du12 +
                    10.0f*_formulaOfD1ry1du11MulD2rx1du12*_d3rx1du13 -
                    10.0f*_d1ry1du11*_d2ry1du12*_d3ry1du13 +
                    _squaredD1ry1du11*_d4ry1du14) +
                _cubedD1rx1du11 *
                (15.0f*_squaredD2rx1du12*_d2ry1du12 -
                    3.0f*_cubedD2ry1du12 +
                    10.0f*_formulaOfD1ry1du11MulD2rx1du12*_d3rx1du13 -
                    10.0f*_d1ry1du11*_d2ry1du12*_d3ry1du13 +
                    2.0f*_squaredD1ry1du11*_d4ry1du14)) /
                Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 4.5f);

        SetD2c1ds12(_d2c1ds12);
    }

    // 曲率計算 ok
    public void CalculateCurvatureOfU2()
    {
        float _cs2;

        _cs2 = -((_squaredD1ry2du21*(_formulaOfD1ry2du21MulD2rx2du22 - _formulaOfD1rx2du21MulD2ry2du22)) / Mathf.Pow(_formulaOfSquaredD1rx2du21PlusSquaredD1ry2du21, 2.5f))
        + (_squaredD1rx2du21*(-(_formulaOfD1ry2du21MulD2rx2du22) + _formulaOfD1rx2du21MulD2ry2du22)) / Mathf.Pow(_formulaOfSquaredD1rx2du21PlusSquaredD1ry2du21, 2.5f);

        SetCs2(_cs2);
    }
    
    // 曲率1次微分 ok
    public void CalculateCurvatureOfU2Derivative1()
    {
        float _d1c2ds21;

        _d1c2ds21 = (_d1rx2du21*((_d1ry2du21*Mathf.Pow(-(_formulaOfD1ry2du21MulD2rx2du22) + _formulaOfD1rx2du21MulD2ry2du22,2)) /
                Mathf.Pow(_formulaOfSquaredD1rx2du21PlusSquaredD1ry2du21, 3.5f) + (-(_cubedD1ry2du21*_squaredD2rx2du22) + _d1rx2du21*_squaredD1ry2du21 *
                    (5.0f*_formulaOfD2rx2du22MulD2ry2du22 - _formulaOfD1ry2du21MulD3rx2du23) - _cubedD1rx2du21*(3.0f*_formulaOfD2rx2du22MulD2ry2du22 + _formulaOfD1ry2du21MulD3rx2du23) +
                    Mathf.Pow(_d1rx2du21, 4)*_d3ry2du23 + _squaredD1rx2du21*_d1ry2du21*(3.0f*_squaredD2rx2du22 - 4.0f*_squaredD2ry2du22 + _formulaOfD1ry2du21MulD3ry2du23)) /
                Mathf.Pow(_formulaOfSquaredD1rx2du21PlusSquaredD1ry2du21, 3.5f))) / Mathf.Sqrt(_formulaOfSquaredD1rx2du21PlusSquaredD1ry2du21) -
                (_d1ry2du21*((_d1rx2du21 *Mathf.Pow(-(_formulaOfD1ry2du21MulD2rx2du22) +_formulaOfD1rx2du21MulD2ry2du22,2)) /
                    Mathf.Pow(_squaredD1rx2du21 +_squaredD1ry2du21, 3.5f) + (_cubedD1ry2du21*(-3*_formulaOfD2rx2du22MulD2ry2du22 + _formulaOfD1ry2du21MulD3rx2du23) +
                        _squaredD1rx2du21*_d1ry2du21*(5.0f*_formulaOfD2rx2du22MulD2ry2du22 + _formulaOfD1ry2du21MulD3rx2du23) -
                        _d1rx2du21*_squaredD1ry2du21*(4.0f*_squaredD2rx2du22 -
                            3.0f*_squaredD2ry2du22 + _formulaOfD1ry2du21MulD3ry2du23) - _cubedD1rx2du21*(_squaredD2ry2du22 + _formulaOfD1ry2du21MulD3ry2du23)) /
                    Mathf.Pow(_formulaOfSquaredD1rx2du21PlusSquaredD1ry2du21, 3.5f))) / Mathf.Sqrt(_formulaOfSquaredD1rx2du21PlusSquaredD1ry2du21);


        SetD1c2ds21(_d1c2ds21);
    }
    
    // 曲率2次微分 ok
    public void CalculateCurvatureOfU2Derivative2()
    {

        float _d2c2ds22;

        _d2c2ds22 = (-(Mathf.Pow(_d1rx2du21, 4) *
                (4.0f*_d2ry2du22*_d3rx2du23 +
                    6.0f*_d2rx2du22*_d3ry2du23 +
                    _d1ry2du21*_d4rx2du24)) +
                _squaredD1rx2du21*_d1ry2du21 *
                (-15.0f*_cubedD2rx2du22 +
                    _d2rx2du22*(39.0f*_squaredD2ry2du22 -
                        2.0f*_formulaOfD1ry2du21MulD3ry2du23) +
                    2.0f*_d1ry2du21*(_d2ry2du22*_d3rx2du23 -
                        _d1ry2du21*_d4rx2du24)) +
                _cubedD1ry2du21 *
                (3.0f*_cubedD2rx2du22 +
                    _d2rx2du22*(-15.0f*_squaredD2ry2du22 +
                        4.0f*_formulaOfD1ry2du21MulD3ry2du23) +
                    _d1ry2du21*(6.0f*_d2ry2du22*_d3rx2du23 -
                        _d1ry2du21*_d4rx2du24)) +
                Mathf.Pow(_d1rx2du21, 5)*_d4ry2du24 +
                _d1rx2du21*_squaredD1ry2du21 *
                (-39.0f*_squaredD2rx2du22*_d2ry2du22 +
                    15.0f*_cubedD2ry2du22 +
                    10.0f*_formulaOfD1ry2du21MulD2rx2du22*_d3rx2du23 -
                    10.0f*_d1ry2du21*_d2ry2du22*_d3ry2du23 +
                    _squaredD1ry2du21*_d4ry2du24) +
                _cubedD1rx2du21 *
                (15.0f*_squaredD2rx2du22*_d2ry2du22 -
                    3.0f*_cubedD2ry2du22 +
                    10.0f*_formulaOfD1ry2du21MulD2rx2du22*_d3rx2du23 -
                    10.0f*_d1ry2du21*_d2ry2du22*_d3ry2du23 +
                    2.0f*_squaredD1ry2du21*_d4ry2du24)) /
                Mathf.Pow(_formulaOfSquaredD1rx2du21PlusSquaredD1ry2du21, 4.5f);

        SetD2c2ds22(_d2c2ds22);
    }

    // 先頭車両追従に関与
    public void CalculateThetaT1()
    {
        float _thetaT1;

        float norm1 = Mathf.Sqrt(_d1rx1du11*_d1rx1du11 + _d1ry1du11*_d1ry1du11);

        _thetaT1 =  Mathf.Atan2( _d1ry1du11 / norm1, _d1rx1du11 / norm1);
        
        // 第一操作点からベジェ曲線へ下ろした時に垂直となる接戦の角度
        SetThetaT1(_thetaT1);
    }

    public void CalculateThetaT2()
    {
        float _thetaT2;

        float norm2 = Mathf.Sqrt(_d1rx2du21*_d1rx2du21 + _d1ry2du21*_d1ry2du21);

        _thetaT2 =  Mathf.Atan2( _d1ry2du21 / norm2, _d1rx2du21 / norm2);
        
        SetThetaT2(_thetaT2);
    }

    public void CalculateThetaP2d()
    {
        float _thetaP2d = Mathf.Atan2(_ry1 - _ry2, _rx1 - _rx2) - thetaT1;

        SetThetaP2d(_thetaP2d);
    }

    // u1のs1の1階微分
    public void CalculateU1DerivativeOfS11()
    {
        float ret = 1f / Mathf.Sqrt(_squaredD1rx1du11 + _squaredD1ry1du11);

        SetD1u1ds11(ret);
    }

    // u1の2階s1微分
    public void CalculateU1DerivativeOfS12()
    {
        float ret;

        ret = -((d1u1ds11*(_d1rx1du11*_d2rx1du12 + _d1ry1du11*_d2ry1du12))/
                    Mathf.Pow(_squaredD1rx1du11 + _squaredD1ry1du11,1.5f));

        SetD2u1ds12(ret);
    }

    // u1の3階s1微分
    public void CalculateU1DerivativeOfS13()
    {
        float ret;

        ret = (-2f*(_squaredD1rx1du11 + _squaredD1ry1du11)*d2u1ds12*
                (_d1rx1du11*_d2rx1du12 + _d1ry1du11*_d2ry1du12) + 
                6f*Mathf.Pow(d1u1ds11,2)*
                Mathf.Pow(_d1rx1du11*_d2rx1du12 + _d1ry1du11*_d2ry1du12,2) - 
                2f*Mathf.Pow(d1u1ds11,2)*
                (_squaredD1rx1du11 + _squaredD1ry1du11)*
                (_squaredD2rx1du12 + _squaredD2ry1du12 + _d1rx1du11*_d3rx1du13 + _d1ry1du11*_d3ry1du13))/
            (2.0f*Mathf.Pow(_squaredD1rx1du11 + _squaredD1ry1du11,2.5f));

        SetD3u1ds13(ret);
    }

    // u2のs2の1階微分
    public void CalculateU2DerivativeOfS21()
    {
        float ret = 1f / Mathf.Sqrt(_squaredD1rx2du21 + _squaredD1ry2du21);

        SetD1u2ds21(ret);
    }

    // u2の2階s2微分
    public void CalculateU2DerivativeOfS22()
    {
        float ret;

        ret = -((d1u2ds21*(_d1rx2du21*_d2rx2du22 + _d1ry2du21*_d2ry2du22))/
                    Mathf.Pow(_squaredD1rx2du21 + _squaredD1ry2du21,1.5f));

        SetD2u2ds22(ret);
    }

    // u2の3階s2微分
    public void CalculateU2DerivativeOfS23()
    {
        float ret;

        ret = (-2f*(_squaredD1rx2du21 + 
                _squaredD1ry2du21)*d2u2ds22*
                (_d1rx2du21*_d2rx2du22 + 
                _d1ry2du21*_d2ry2du22) + 
                6f*Mathf.Pow(d1u2ds21,2)*
                Mathf.Pow(_d1rx2du21*_d2rx2du22 + 
                _d1ry2du21*_d2ry2du22,2) - 
                2f*Mathf.Pow(d1u2ds21,2)*
                (_squaredD1rx2du21 + 
                _squaredD1ry2du21)*
                (_squaredD2rx2du22 + 
                _squaredD2ry2du22 + 
                _d1rx2du21*_d3rx2du23 + 
                _d1ry2du21*_d3ry2du23))/
            (2.0f*Mathf.Pow(_squaredD1rx2du21 + 
                _squaredD1ry2du21,2.5f));

        SetD3u2ds23(ret);
    }

    // thetaT1の一階s1微分 = 曲率
    public void CalculateThetaT1DerivativeOfS11()
    {
        SetD1thetaT1ds11(cs1);
    }

    // thetaT1の2階s1微分 = 曲率の一階微分
    public void CalculateThetaT1DerivativeOfS12()
    {
        SetD2thetaT1ds12(d1c1ds11);
    }

    // thetaT1の3階s1微分 = 曲率の2回微分
    public void CalculateThetaT1DerivativeOfS13()
    {
        SetD3thetaT1ds13(d2c1ds12);
    }

    // thetaT2の一階s2微分 = 曲率
    public void CalculateThetaT2DerivativeOfS21()
    {
        SetD1thetaT2ds21(cs2);
    }

    // thetaT2の2階s2微分 = 曲率の一階微分
    public void CalculateThetaT2DerivativeOfS22()
    {
        SetD2thetaT2ds22(d1c2ds21);
    }

    // thetaT2の3階s2微分 = 曲率の2回微分
    public void CalculateThetaT2DerivativeOfS23()
    {
        SetD3thetaT2ds23(d2c2ds22);
    }

    // s2の1階s1微分
    public void CalculateS2DerivativeOfS11()
    {
        float ret;

        ret = _cosThetaP2d*_secThetaP2dPlusThetaT1MinusThetaT2;

        SetD1s2ds11(ret);
    }

    // s2の2階s1微分
    public void CalculateS2DerivativeOfS12()
    {
        float ret;

        ret =  -(_secThetaP2dPlusThetaT1MinusThetaT2*_sinThetaP2d*d1thetaP2dds11) + 
            _cosThetaP2d*_secThetaP2dPlusThetaT1MinusThetaT2*_tanThetaP2dPlusThetaT1MinusThetaT2*
            (d1thetaP2dds11 + d1thetaT1ds11 -  d1s2ds11*d1thetaT2ds21);

        SetD2s2ds12(ret);
    }

    // s2の3階s1微分
    public void CalculateS2DerivativeOfS13()
    {
        float ret;

        // replaceで置換
        ret = -(_cosThetaP2d*_secThetaP2dPlusThetaT1MinusThetaT2*
                Mathf.Pow(d1thetaP2dds11,2)) - 
            2f*_secThetaP2dPlusThetaT1MinusThetaT2*_sinThetaP2d*
            _tanThetaP2dPlusThetaT1MinusThetaT2*
            d1thetaP2dds11*
            (d1thetaP2dds11 + d1thetaT1ds11 - 
                d1s2ds11*d1thetaT2ds21) + 
            _cosThetaP2d*Mathf.Pow(_secThetaP2dPlusThetaT1MinusThetaT2,3)*Mathf.Pow(d1thetaP2dds11 + 
                d1thetaT1ds11 - 
                d1s2ds11*d1thetaT2ds21,2) + 
            _cosThetaP2d*_secThetaP2dPlusThetaT1MinusThetaT2*
            Mathf.Pow(_tanThetaP2dPlusThetaT1MinusThetaT2,2)*
            Mathf.Pow(d1thetaP2dds11 + d1thetaT1ds11 - 
                d1s2ds11*d1thetaT2ds21,2) - 
            _secThetaP2dPlusThetaT1MinusThetaT2*_sinThetaP2d*
            d2thetaP2dds12 + 
            _cosThetaP2d*_secThetaP2dPlusThetaT1MinusThetaT2*
            _tanThetaP2dPlusThetaT1MinusThetaT2*
            (-(d1thetaT2ds21*d2s2ds12) + 
                d2thetaP2dds12 + d2thetaT1ds12 - 
                Mathf.Pow(d1s2ds11,2)*d2thetaT2ds22);

        SetD3s2ds13(ret);
    }

    // u2の1階s1微分
    public void CalculateU2DerivativeOfS11()
    {
        float ret;

        ret = d1u2ds21*d1s2ds11;

        SetD1u2ds11(ret);
    }

    // u2の2階s1微分
    public void CalculateU2DerivativeOfS12()
    {
        float ret;
        ret =  d1s2ds11*d2u2ds22 + d1u2ds21*d2s2ds12;

        SetD2u2ds12(ret);
    }

    // u2の3階s1微分
    public void CalculateU2DerivativeOfS13()
    {
        float ret;

        ret =  2f*d2u2ds22*d2s2ds12 + d1s2ds11*d3u2ds23 + d1u2ds21*d3s2ds13;

        SetD3u2ds13(ret);
    }

    // thetaP2dのs1の1階微分
    public void CalculateThetaP2dDerivativeOfS11()
    {

        float ret;

        ret = (-((_formulaOfRy1SubRy2)*
                            (d1u1ds11*_d1rx1du11 - 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11)) + 
                        (_formulaOfRx1SubRx2)*
                        (d1u1ds11*_d1ry1du11 - 
                            d1u2ds21*_d1ry2du21*
                            d1s2ds11))/
                    (_squaredFormulaOfRx1SubRx2*
                        (1 + _squaredFormulaOfRy1SubRy2/
                            _squaredFormulaOfRx1SubRx2)) - d1thetaT1ds11;

        SetD1thetaP2dds11(ret);
    }

    // thetaP2dのs1の２階微分
    public void CalculateThetaP2dDerivativeOfS12()
    {
        float ret;

        ret = (2*(d1u1ds11*_d1rx1du11 - 
                            d1u2ds21*_d1rx2du21*
                                d1s2ds11)*
                            ((_formulaOfRy1SubRy2)*
                                (d1u1ds11*_d1rx1du11 - 
                                d1u2ds21*_d1rx2du21*
                                d1s2ds11) - 
                            (_formulaOfRx1SubRx2)*
                                (d1u1ds11*_d1ry1du11 - 
                                d1u2ds21*_d1ry2du21*
                                d1s2ds11)))/
                        ((_formulaOfRx1SubRx2)*
                            (_squaredFormulaOfRx1SubRx2 + 
                            _squaredFormulaOfRy1SubRy2)) - 
                        (2*(_formulaOfRy1SubRy2)*
                            Mathf.Pow(_ry1*(d1u1ds11*
                                _d1rx1du11 - 
                                d1u2ds21*_d1rx2du21*
                                d1s2ds11) + 
                            _ry2*(-(d1u1ds11*
                                    _d1rx1du11) + 
                                d1u2ds21*_d1rx2du21*
                                d1s2ds11) - 
                            (_formulaOfRx1SubRx2)*
                                (d1u1ds11*_d1ry1du11 - 
                                d1u2ds21*_d1ry2du21*
                                d1s2ds11),2))/
                        ((_formulaOfRx1SubRx2)*
                            Mathf.Pow(_squaredRx1 - 2*_rx1*_rx2 + 
                            _squaredRx2 + _squaredFormulaOfRy1SubRy2,2))  
                        - ((_formulaOfRy1SubRy2)*
                            (_d1rx1du11*d2u1ds12 + 
                                Mathf.Pow(d1u1ds11,2)*_d2rx1du12 - 
                                Mathf.Pow(d1u2ds21,2)*
                                Mathf.Pow(d1s2ds11,2)*_d2rx2du22 - 
                                _d1rx2du21*
                                (Mathf.Pow(d1s2ds11,2)*d2u2ds22 + 
                                d1u2ds21*d2s2ds12)) - 
                            (_formulaOfRx1SubRx2)*
                            (_d1ry1du11*d2u1ds12 + 
                                Mathf.Pow(d1u1ds11,2)*_d2ry1du12 - 
                                Mathf.Pow(d1u2ds21,2)*Mathf.Pow(d1s2ds11,2)*
                                _d2ry2du22 - 
                                _d1ry2du21*
                                (Mathf.Pow(d1s2ds11,2)*d2u2ds22 + 
                                d1u2ds21*d2s2ds12)))/
                        (_squaredFormulaOfRx1SubRx2 + 
                            _squaredFormulaOfRy1SubRy2) - d2thetaT1ds12;

        SetD2thetaP2dds12(ret);
    }

    // thetaP2dのs1の３階微分
    public void CalculateThetaP2dDerivativeOfS13()
    {

        float ret;
        
        ret = (-2*Mathf.Pow(d1u1ds11*_d1rx1du11 - 
                        d1u2ds21*_d1rx2du21*
                            d1s2ds11,2)*
                        ((_formulaOfRy1SubRy2)*
                            (d1u1ds11*_d1rx1du11 - 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) - 
                        (_formulaOfRx1SubRx2)*
                            (d1u1ds11*_d1ry1du11 - 
                            d1u2ds21*_d1ry2du21*
                            d1s2ds11)))/
                    (_squaredFormulaOfRx1SubRx2*
                        (_squaredFormulaOfRx1SubRx2 + 
                        _squaredFormulaOfRy1SubRy2)) + 
                    (2*(_formulaOfRy1SubRy2)*
                        (d1u1ds11*_d1rx1du11 - 
                        d1u2ds21*_d1rx2du21*
                            d1s2ds11)*
                        Mathf.Pow(_ry1*(d1u1ds11*
                            _d1rx1du11 - 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) + 
                        _ry2*(-(d1u1ds11*
                                _d1rx1du11) + 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) - 
                        (_formulaOfRx1SubRx2)*
                            (d1u1ds11*_d1ry1du11 - 
                            d1u2ds21*_d1ry2du21*
                            d1s2ds11),2))/
                    (_squaredFormulaOfRx1SubRx2*
                        Mathf.Pow(_squaredRx1 - 2*_rx1*_rx2 + 
                        _squaredRx2 + _squaredFormulaOfRy1SubRy2,2))  
                    - (2*(d1u1ds11*_d1ry1du11 - 
                        d1u2ds21*_d1ry2du21*
                            d1s2ds11)*
                        Mathf.Pow(_ry2*(d1u1ds11*
                            _d1rx1du11 - 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) + 
                        _ry1*(-(d1u1ds11*
                                _d1rx1du11) + 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) + 
                        (_formulaOfRx1SubRx2)*
                            (d1u1ds11*_d1ry1du11 - 
                            d1u2ds21*_d1ry2du21*
                            d1s2ds11),2))/
                    ((_formulaOfRx1SubRx2)*
                        Mathf.Pow(_squaredRx1 - 2*_rx1*_rx2 + 
                        _squaredRx2 + _squaredFormulaOfRy1SubRy2,2))  
                    + (8*(_formulaOfRy1SubRy2)*
                        Mathf.Pow(_ry1*(d1u1ds11*
                            _d1rx1du11 - 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) + 
                        _ry2*(-(d1u1ds11*
                                _d1rx1du11) + 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) - 
                        (_formulaOfRx1SubRx2)*
                            (d1u1ds11*_d1ry1du11 - 
                            d1u2ds21*_d1ry2du21*
                            d1s2ds11),2)*
                        (_rx1*(d1u1ds11*_d1rx1du11 - 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) + 
                        _rx2*(-(d1u1ds11*
                                _d1rx1du11) + 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) + 
                        (_formulaOfRy1SubRy2)*
                            (d1u1ds11*_d1ry1du11 - 
                            d1u2ds21*_d1ry2du21*
                            d1s2ds11)))/
                    ((_formulaOfRx1SubRx2)*
                        Mathf.Pow(_squaredRx1 - 2*_rx1*_rx2 + 
                        _squaredRx2 + _squaredFormulaOfRy1SubRy2,3))  
                    - (2*(d1u1ds11*_d1rx1du11 - 
                        d1u2ds21*_d1rx2du21*
                            d1s2ds11)*
                        ((_formulaOfRy1SubRy2)*
                            (d1u1ds11*_d1rx1du11 - 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) - 
                        (_formulaOfRx1SubRx2)*
                            (d1u1ds11*_d1ry1du11 - 
                            d1u2ds21*_d1ry2du21*
                            d1s2ds11))*
                        (2*(_formulaOfRx1SubRx2)*
                            (d1u1ds11*_d1rx1du11 - 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) + 
                        2*(_formulaOfRy1SubRy2)*
                            (d1u1ds11*_d1ry1du11 - 
                            d1u2ds21*_d1ry2du21*
                            d1s2ds11)))/
                    ((_formulaOfRx1SubRx2)*
                        Mathf.Pow(_squaredFormulaOfRx1SubRx2 + 
                        _squaredFormulaOfRy1SubRy2,2)) + 
                    (2*((_formulaOfRy1SubRy2)*
                            (d1u1ds11*_d1rx1du11 - 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) - 
                        (_formulaOfRx1SubRx2)*
                            (d1u1ds11*_d1ry1du11 - 
                            d1u2ds21*_d1ry2du21*
                            d1s2ds11))*
                        (_d1rx1du11*d2u1ds12 + 
                        Mathf.Pow(d1u1ds11,2)*_d2rx1du12 - 
                        Mathf.Pow(d1u2ds21,2)*Mathf.Pow(d1s2ds11,2)*
                            _d2rx2du22 - 
                        _d1rx2du21*
                            (Mathf.Pow(d1s2ds11,2)*d2u2ds22 + 
                            d1u2ds21*d2s2ds12)))/
                    ((_formulaOfRx1SubRx2)*
                        (_squaredFormulaOfRx1SubRx2 + 
                        _squaredFormulaOfRy1SubRy2)) + 
                    (2*(d1u1ds11*_d1rx1du11 - 
                        d1u2ds21*_d1rx2du21*
                            d1s2ds11)*
                        ((_formulaOfRy1SubRy2)*
                            (_d1rx1du11*d2u1ds12 + 
                            Mathf.Pow(d1u1ds11,2)*_d2rx1du12 - 
                            Mathf.Pow(d1u2ds21,2)*
                            Mathf.Pow(d1s2ds11,2)*_d2rx2du22  
                    - _d1rx2du21*
                            (Mathf.Pow(d1s2ds11,2)*d2u2ds22 + 
                                d1u2ds21*d2s2ds12)) - 
                        (_formulaOfRx1SubRx2)*
                            (_d1ry1du11*d2u1ds12 + 
                            Mathf.Pow(d1u1ds11,2)*_d2ry1du12 - 
                            Mathf.Pow(d1u2ds21,2)*
                            Mathf.Pow(d1s2ds11,2)*_d2ry2du22 - 
                            _d1ry2du21*
                            (Mathf.Pow(d1s2ds11,2)*d2u2ds22 + 
                                d1u2ds21*d2s2ds12))))/
                    ((_formulaOfRx1SubRx2)*
                        (_squaredFormulaOfRx1SubRx2 + 
                        _squaredFormulaOfRy1SubRy2)) + 
                    ((2*(_formulaOfRx1SubRx2)*
                            (d1u1ds11*_d1rx1du11 - 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) + 
                        2*(_formulaOfRy1SubRy2)*
                            (d1u1ds11*_d1ry1du11 - 
                            d1u2ds21*_d1ry2du21*
                            d1s2ds11))*
                        ((_formulaOfRy1SubRy2)*
                            (_d1rx1du11*d2u1ds12 + 
                            Mathf.Pow(d1u1ds11,2)*_d2rx1du12 - 
                            Mathf.Pow(d1u2ds21,2)*
                            Mathf.Pow(d1s2ds11,2)*_d2rx2du22  
                    - _d1rx2du21*
                            (Mathf.Pow(d1s2ds11,2)*d2u2ds22 + 
                                d1u2ds21*d2s2ds12)) - 
                        (_formulaOfRx1SubRx2)*
                            (_d1ry1du11*d2u1ds12 + 
                            Mathf.Pow(d1u1ds11,2)*_d2ry1du12 - 
                            Mathf.Pow(d1u2ds21,2)*
                            Mathf.Pow(d1s2ds11,2)*_d2ry2du22 - 
                            _d1ry2du21*
                            (Mathf.Pow(d1s2ds11,2)*d2u2ds22 + 
                                d1u2ds21*d2s2ds12))))/
                    Mathf.Pow(_squaredFormulaOfRx1SubRx2 + 
                        _squaredFormulaOfRy1SubRy2,2) - 
                    (4*(_formulaOfRy1SubRy2)*
                        (_ry1*(d1u1ds11*_d1rx1du11 - 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) + 
                        _ry2*(-(d1u1ds11*
                                _d1rx1du11) + 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11) - 
                        (_formulaOfRx1SubRx2)*
                            (d1u1ds11*_d1ry1du11 - 
                            d1u2ds21*_d1ry2du21*
                            d1s2ds11))*
                        (_ry1*(_d1rx1du11*d2u1ds12 + 
                            Mathf.Pow(d1u1ds11,2)*_d2rx1du12 - 
                            Mathf.Pow(d1u2ds21,2)*
                            Mathf.Pow(d1s2ds11,2)*_d2rx2du22  
                    - _d1rx2du21*
                            (Mathf.Pow(d1s2ds11,2)*d2u2ds22 + 
                                d1u2ds21*d2s2ds12)) + 
                        _ry2*(-(_d1rx1du11*
                                d2u1ds12) - 
                            Mathf.Pow(d1u1ds11,2)*_d2rx1du12 + 
                            Mathf.Pow(d1u2ds21,2)*
                            Mathf.Pow(d1s2ds11,2)*_d2rx2du22  
                    + _d1rx2du21*
                            (Mathf.Pow(d1s2ds11,2)*d2u2ds22 + 
                                d1u2ds21*d2s2ds12)) - 
                        (_formulaOfRx1SubRx2)*
                            (_d1ry1du11*d2u1ds12 + 
                            Mathf.Pow(d1u1ds11,2)*_d2ry1du12 - 
                            Mathf.Pow(d1u2ds21,2)*
                            Mathf.Pow(d1s2ds11,2)*_d2ry2du22 - 
                            _d1ry2du21*
                            (Mathf.Pow(d1s2ds11,2)*d2u2ds22 + 
                                d1u2ds21*d2s2ds12))))/
                    ((_formulaOfRx1SubRx2)*
                        Mathf.Pow(_squaredRx1 - 2*_rx1*_rx2 + 
                        _squaredRx2 + _squaredFormulaOfRy1SubRy2,2))  
                    - ((d1u1ds11*_d1ry1du11 - 
                            d1u2ds21*_d1ry2du21*
                            d1s2ds11)*
                        (_d1rx1du11*d2u1ds12 + 
                            Mathf.Pow(d1u1ds11,2)*_d2rx1du12 - 
                            Mathf.Pow(d1u2ds21,2)*
                            Mathf.Pow(d1s2ds11,2)*_d2rx2du22 - 
                            _d1rx2du21*
                            (Mathf.Pow(d1s2ds11,2)*d2u2ds22 + 
                            d1u2ds21*d2s2ds12)) - 
                        (d1u1ds11*_d1rx1du11 - 
                            d1u2ds21*_d1rx2du21*
                            d1s2ds11)*
                        (_d1ry1du11*d2u1ds12 + 
                            Mathf.Pow(d1u1ds11,2)*_d2ry1du12 - 
                            Mathf.Pow(d1u2ds21,2)*
                            Mathf.Pow(d1s2ds11,2)*_d2ry2du22 - 
                            _d1ry2du21*
                            (Mathf.Pow(d1s2ds11,2)*d2u2ds22 + 
                            d1u2ds21*d2s2ds12)) + 
                        (_formulaOfRy1SubRy2)*
                        (3*d1u1ds11*d2u1ds12*
                            _d2rx1du12 - 
                            3*_d1rx2du21*d1s2ds11*
                            d2u2ds22*d2s2ds12 - 
                            3*Mathf.Pow(d1u2ds21,2)*d1s2ds11*
                            _d2rx2du22*d2s2ds12 + 
                            _d1rx1du11*d3u1ds13 - 
                            _d1rx2du21*Mathf.Pow(d1s2ds11,3)*
                            d3u2ds23 + 
                            Mathf.Pow(d1u1ds11,3)*_d3rx1du13 - 
                            Mathf.Pow(d1u2ds21,3)*
                            Mathf.Pow(d1s2ds11,3)*_d3rx2du23 - 
                            d1u2ds21*
                            (3*Mathf.Pow(d1s2ds11,3)*d2u2ds22*
                                _d2rx2du22 + 
                            _d1rx2du21*d3s2ds13)) - 
                        (_formulaOfRx1SubRx2)*
                        (3*d1u1ds11*d2u1ds12*
                            _d2ry1du12 - 
                            3*_d1ry2du21*d1s2ds11*
                            d2u2ds22*d2s2ds12 - 
                            3*Mathf.Pow(d1u2ds21,2)*d1s2ds11*
                            _d2ry2du22*d2s2ds12 + 
                            _d1ry1du11*d3u1ds13 - 
                            _d1ry2du21*Mathf.Pow(d1s2ds11,3)*
                            d3u2ds23 + 
                            Mathf.Pow(d1u1ds11,3)*_d3ry1du13 - 
                            Mathf.Pow(d1u2ds21,3)*Mathf.Pow(d1s2ds11,3)*
                            _d3ry2du23 - 
                            d1u2ds21*
                            (3*Mathf.Pow(d1s2ds11,3)*d2u2ds22*
                                _d2ry2du22 + 
                            _d1ry2du21*d3s2ds13)))/
                    (_squaredFormulaOfRx1SubRx2 + 
                        _squaredFormulaOfRy1SubRy2) - d3thetaT1ds13;

        SetD3thetaP2dds13(ret);
    }


    // Setter
    public void SetCs1(float v) { cs1 = v; }
    public void SetD1c1ds11(float v) { d1c1ds11 = v; }
    public void SetD2c1ds12(float v) { d2c1ds12 = v; }

    public void SetCs2(float v) { cs2 = v; }
    public void SetD1c2ds21(float v) { d1c2ds21 = v; }
    public void SetD2c2ds22(float v) { d2c2ds22 = v; }

    public void SetThetaT1(float v) { thetaT1 = v; }
    public void SetThetaT2(float v) { thetaT2 = v; }
    public void SetThetaP2d(float v) { thetaP2d = v; }
    // u1(s1) 系
    public void SetD1u1ds11(float v) { d1u1ds11 = v; }
    public void SetD2u1ds12(float v) { d2u1ds12 = v; }
    public void SetD3u1ds13(float v) { d3u1ds13 = v; }
    // u2(s2) 系
    public void SetD1u2ds21(float v) { d1u2ds21 = v; }
    public void SetD2u2ds22(float v) { d2u2ds22 = v; }
    public void SetD3u2ds23(float v) { d3u2ds23 = v; }
    // u2(s1) 系
    public void SetD1u2ds11(float v) { d1u2ds11 = v; }
    public void SetD2u2ds12(float v) { d2u2ds12 = v; }
    public void SetD3u2ds13(float v) { d3u2ds13 = v; }
    // θT1 系
    public void SetD1thetaT1ds11(float v) { d1thetaT1ds11 = v; }
    public void SetD2thetaT1ds12(float v) { d2thetaT1ds12 = v; }
    public void SetD3thetaT1ds13(float v) { d3thetaT1ds13 = v; }
    // θT2 系
    public void SetD1thetaT2ds21(float v) { d1thetaT2ds21 = v; }
    public void SetD2thetaT2ds22(float v) { d2thetaT2ds22 = v; }
    public void SetD3thetaT2ds23(float v) { d3thetaT2ds23 = v; }
    // s2(s1) 系
    public void SetD1s2ds11(float v) { d1s2ds11 = v; }
    public void SetD2s2ds12(float v) { d2s2ds12 = v; }
    public void SetD3s2ds13(float v) { d3s2ds13 = v; }
    // θP2d 系
    public void SetD1thetaP2dds11(float v) { d1thetaP2dds11 = v; }
    public void SetD2thetaP2dds12(float v) { d2thetaP2dds12 = v; }
    public void SetD3thetaP2dds13(float v) { d3thetaP2dds13 = v; }

    // Getter
    public float GetCs1() => cs1;
    public float GetD1c1ds11() => d1c1ds11;
    public float GetD2c1ds12() => d2c1ds12;

    public float GetCs2() => cs2;
    public float GetD1c2ds21() => d1c2ds21;
    public float GetD2c2ds22() => d2c2ds22;

    public float GetThetaT1() => thetaT1;
    public float GetThetaT2() => thetaT2;
    public float GetThetaP2d() => thetaP2d;
    
    // u1 系
    public float GetD1u1ds11() => d1u1ds11;
    public float GetD2u1ds12() => d2u1ds12;
    public float GetD3u1ds13() => d3u1ds13;
    // u2(s2) 系
    public float GetD1u2ds21() => d1u2ds21;
    public float GetD2u2ds22() => d2u2ds22;
    public float GetD3u2ds23() => d3u2ds23;
    // u2(s1) 系
    public float GetD1u2ds11() => d1u2ds11;
    public float GetD2u2ds12() => d2u2ds12;
    public float GetD3u2ds13() => d3u2ds13;
    // θT1 系
    public float GetD1thetaT1ds11() => d1thetaT1ds11;
    public float GetD2thetaT1ds12() => d2thetaT1ds12;
    public float GetD3thetaT1ds13() => d3thetaT1ds13;
    // θT2 系
    public float GetD1thetaT2ds21() => d1thetaT2ds21;
    public float GetD2thetaT2ds22() => d2thetaT2ds22;
    public float GetD3thetaT2ds23() => d3thetaT2ds23;
    // s2(s1) 系
    public float GetD1s2ds11() => d1s2ds11;
    public float GetD2s2ds12() => d2s2ds12;
    public float GetD3s2ds13() => d3s2ds13;
    // θP2d 系
    public float GetD1thetaP2dds11() => d1thetaP2dds11;
    public float GetD2thetaP2dds12() => d2thetaP2dds12;
    public float GetD3thetaP2dds13() => d3thetaP2dds13;


    private void ComputeUtilitiesForBezierGeometry()
    {
        _rx1 = bsplineGeometry.GetRx1();
        _ry1 = bsplineGeometry.GetRy1();

        _d1rx1du11 = bsplineGeometry.GetD1Rx1du11();
        _d1ry1du11 = bsplineGeometry.GetD1Ry1du11();
        _d2rx1du12 = bsplineGeometry.GetD2Rx1du12();
        _d2ry1du12 = bsplineGeometry.GetD2Ry1du12();
        _d3rx1du13 = bsplineGeometry.GetD3Rx1du13();
        _d3ry1du13 = bsplineGeometry.GetD3Ry1du13();
        _d4rx1du14 = bsplineGeometry.GetD4Rx1du14();
        _d4ry1du14 = bsplineGeometry.GetD4Ry1du14();

        _squaredRx1 = Mathf.Pow(_rx1,2);
        _squaredRy1 = Mathf.Pow(_ry1,2);

        _squaredD1rx1du11 = Mathf.Pow(_d1rx1du11,2);
        _cubedD1rx1du11 = Mathf.Pow(_d1rx1du11,3);
        _squaredD1ry1du11 = Mathf.Pow(_d1ry1du11,2);
        _cubedD1ry1du11 = Mathf.Pow(_d1ry1du11,3);

        _squaredD2rx1du12 = Mathf.Pow(_d2rx1du12,2);
        _cubedD2rx1du12 = Mathf.Pow(_d2rx1du12,3);
        _squaredD2ry1du12 = Mathf.Pow(_d2ry1du12,2);
        _cubedD2ry1du12 = Mathf.Pow(_d2ry1du12,3);

        _formulaOfD1ry1du11MulD2rx1du12 = _d1ry1du11*_d2rx1du12;
        _formulaOfD1ry1du11MulD3rx1du13 = _d1ry1du11*_d3rx1du13;
        _formulaOfD1ry1du11MulD3ry1du13 = _d1ry1du11*_d3ry1du13;
        _formulaOfD1rx1du11MulD2ry1du12 = _d1rx1du11*_d2ry1du12;
        _formulaOfD2rx1du12MulD2ry1du12 = _d2rx1du12*_d2ry1du12;

        _formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11 = _squaredD1rx1du11 + _squaredD1ry1du11;


        ///////
        _rx2 = bsplineGeometry.GetRx2();
        _ry2 = bsplineGeometry.GetRy2();

        _d1rx2du21 = bsplineGeometry.GetD1Rx2du21();
        _d1ry2du21 = bsplineGeometry.GetD1Ry2du21();
        _d2rx2du22 = bsplineGeometry.GetD2Rx2du22();
        _d2ry2du22 = bsplineGeometry.GetD2Ry2du22();
        _d3rx2du23 = bsplineGeometry.GetD3Rx2du23();
        _d3ry2du23 = bsplineGeometry.GetD3Ry2du23();
        _d4rx2du24 = bsplineGeometry.GetD4Rx2du24();
        _d4ry2du24 = bsplineGeometry.GetD4Ry2du24();

        _squaredRx2 = Mathf.Pow(_rx2,2);
        _squaredRy2 = Mathf.Pow(_ry2,2);

        _squaredD1rx2du21 = Mathf.Pow(_d1rx2du21,2);
        _cubedD1rx2du21 = Mathf.Pow(_d1rx2du21,3);
        _squaredD1ry2du21 = Mathf.Pow(_d1ry2du21,2);
        _cubedD1ry2du21 = Mathf.Pow(_d1ry2du21,3);

        _squaredD2rx2du22 = Mathf.Pow(_d2rx2du22,2);
        _cubedD2rx2du22 = Mathf.Pow(_d2rx2du22,3);
        _squaredD2ry2du22 = Mathf.Pow(_d2ry2du22,2);
        _cubedD2ry2du22 = Mathf.Pow(_d2ry2du22,3);

        _formulaOfD1ry2du21MulD2rx2du22 = _d1ry2du21*_d2rx2du22;
        _formulaOfD1ry2du21MulD3rx2du23 = _d1ry2du21*_d3rx2du23;
        _formulaOfD1ry2du21MulD3ry2du23 = _d1ry2du21*_d3ry2du23;
        _formulaOfD1rx2du21MulD2ry2du22 = _d1rx2du21*_d2ry2du22;
        _formulaOfD2rx2du22MulD2ry2du22 = _d2rx2du22*_d2ry2du22;

        _formulaOfSquaredD1rx2du21PlusSquaredD1ry2du21 = _squaredD1rx2du21 + _squaredD1ry2du21;


        _formulaOfRx1SubRx2 = _rx1 - _rx2;
        _formulaOfRy1SubRy2 = _ry1 - _ry2;

        _squaredFormulaOfRx1SubRx2 = Mathf.Pow(_formulaOfRx1SubRx2,2);
        _squaredFormulaOfRy1SubRy2 = Mathf.Pow(_formulaOfRy1SubRy2,2);
    }

    private void ComputeUtilitiesForPathKinematics()
    {
        _sinThetaP2d = Mathf.Sin(thetaP2d);
        _cosThetaP2d = Mathf.Cos(thetaP2d);

        // Debug.Log($"thetaP2d:{thetaP2d}, _sinThetaP2d:{_sinThetaP2d}, _cosThetaP2d:{_cosThetaP2d}");

        _formulaOfThetaP2dPlusThetaT1MinusThetaT2 = thetaP2d + thetaT1 - thetaT2;
        _secThetaP2dPlusThetaT1MinusThetaT2 = 1.0f / Mathf.Cos(_formulaOfThetaP2dPlusThetaT1MinusThetaT2);
        _tanThetaP2dPlusThetaT1MinusThetaT2 = Mathf.Tan(_formulaOfThetaP2dPlusThetaT1MinusThetaT2);
    }
}
