using Unity.Mathematics;
using UnityEngine;
using System.IO;
using System.Text;
using System.Globalization;
using System;
using Unity.Profiling;
using Unity.Jobs;
using Unity.Collections;


public class BsplineGeometry
{
    static ProfilerMarker bsplineGeometryMarker =
        new ProfilerMarker("BsplineGeometry.CalculationBsplineGeometry()");
    static ProfilerMarker GeneratePointsMarker = new ProfilerMarker("BSplineCurve.generatePoints");
    static ProfilerMarker DerivativePointsMarker = new ProfilerMarker("BSplineCurve.DerivativeAllPoints()");


    private PsFinder psFinder;
    public TrajectoryGenerator trajectoryGenerator;

    //　制御点データ
    float2[] controlPoints;
    // ノットベクトル
    float[] knots;

    //　Bスプライン曲線 R(u)
    // 200,0000点離散点を保持するのは難しいため、Ps探索に必要な範囲だけに切り抜く
    // 先頭車両のPs探索用 離散点 5[m]
    public float2[] frontPoints;
    // 後方車両のPs探索用 離散点 5[m]
    public float2[] rearPoints;
    // 1階微分　dR(u)/du
    public float2[] frontDerivative1;
    public float2[] rearDerivative1;


    private int Nctrl; // 制御点の総数　1202
    private int N;// 120100
    private int Ns;
    public float ds;
    private int k;
    private int p; 
    private int n;
    private int m;   // ノット数　可変
    public float[] u;
    private float u_min;
    private float u_max;

    private float rx1, ry1;
    private float d1Rx1du11, d1Ry1du11;
    private float d2Rx1du12, d2Ry1du12;
    private float d3Rx1du13, d3Ry1du13;
    private float d4Rx1du14, d4Ry1du14;

    private float rx2, ry2;
    private float d1Rx2du21, d1Ry2du21;
    private float d2Rx2du22, d2Ry2du22;
    private float d3Rx2du23, d3Ry2du23;
    private float d4Rx2du24, d4Ry2du24;

    private float[,] Nmat, dbMat1, dbMat2, dbMat3, dbMat4;
    private int frontStart, frontEnd;
    private int rearStart, rearEnd;


    public BsplineGeometry()
    {
    }

    public void Initialize(TrajectoryGenerator trajectoryGenerator, PsFinder finder)
    {
        this.trajectoryGenerator = trajectoryGenerator;
        this.psFinder = finder;




        // controlPoints = new NativeArray<float2>(trajectoryGenerator.GetNctrl(), Allocator.Persistent);
        Nctrl = trajectoryGenerator.GetNctrl();
        controlPoints = new float2[Nctrl];

        k = trajectoryGenerator.GetK() + 1;
        p = k - 1;               
        n = trajectoryGenerator.GetNctrl() - 1;      // ← 1201
        m = n + k + 1; // 1001

        N = trajectoryGenerator.GetN(); // 20,000
        // 部分的に切り抜くため
        Ns = 8001;

        frontStart = N - Ns; // 15,000
        frontEnd   = frontStart + Ns; // 20,001

        rearStart = 2000;
        rearEnd = rearStart + Ns; // 11,0001

        // t_ver = 0:9
        float[] t_ver = new float[Nctrl];

        for (int i = 0; i < Nctrl; i++)
            t_ver[i] = i;

        // ノットベクトルの算出
        knots = GenerateKnots(Nctrl, p, t_ver);

        // 曲線を細かくサンプリングするパラメータの生成
        float[] t_sampling = new float[N];
        for (int i = 0; i < N; i++)
            t_sampling[i] = math.lerp(t_ver[0], t_ver[^1], (float)i / (N - 1));

        // Bスプライン基底関数行列の算出
        // Ns x n行列
        Debug.Log("CalculateBsplineBasisMatrix(k, t_sampling, knots) Start");
        Nmat = CalculateBsplineBasisMatrix(k, t_sampling, knots);

        Debug.Log("CalculateBsplineBasisDerivativeMatrix() 1階微分 Start");
        int order = 1;
        dbMat1 = CalculateBsplineBasisDerivativeMatrix(k, knots, t_sampling, order);

        Debug.Log("CalculateBsplineBasisDerivativeMatrix() 2階微分 Start");
        order = 2;
        dbMat2 = CalculateBsplineBasisDerivativeMatrix(k, knots, t_sampling, order);

        Debug.Log("CalculateBsplineBasisDerivativeMatrix() 3階微分 Start");
        order = 3;
        dbMat3 = CalculateBsplineBasisDerivativeMatrix(k, knots, t_sampling, order);

        Debug.Log("CalculateBsplineBasisDerivativeMatrix() 4階微分 Start");
        order = 4;
        dbMat4 = CalculateBsplineBasisDerivativeMatrix(k, knots, t_sampling, order);

        frontPoints = new float2[Ns];
        rearPoints = new float2[Ns];

        frontDerivative1 = new float2[Ns];
        rearDerivative1 = new float2[Ns];

        ds = trajectoryGenerator.GetDs();

        u_min = knots[k];
        u_max = knots[n + 1];

        u = CaluculateArrayU();

        Debug.Log($"u_min:{u_min}, u_max:{u_max}");

        Debug.Log("BsplineGeometry Initialize 完了");
    }

    // pastデータ近似しなくて良くないか
    // 離散点列データは一部だけで良い説
    // Ps探索だけに使う
    public void Approximate(float2[] input)
    {

        // Debug.Log("EvaluateBspline(Nmat, controlPoints) Start");
        // input が制御点なら、まず controlPoints に反映
        if (input.Length != Nctrl) Debug.LogError($"input.Length={input.Length} Nctrl={Nctrl}");
        Array.Copy(input, controlPoints, Nctrl);

        frontPoints = EvaluateBspline(Nmat, frontStart, frontEnd);
        rearPoints = EvaluateBspline(Nmat, rearStart, rearEnd);

        return;
    }


    public void Derivative1AllPoints()
    {

        // Debug.Log("DerivativeBspline(dbMat1, controlPoints) 1階微分 Start");
        frontDerivative1 = DerivativeBspline(dbMat1, frontStart, frontEnd);
        rearDerivative1 = DerivativeBspline(dbMat1, rearStart, rearEnd);

        return;
    }

    public void CalculationBsplineGeometry()
    {

        using (bsplineGeometryMarker.Auto())
        {

            // Ps探索で見つけたインデックスを取得
            // int idx = PsFinder.u1Index;
            int u1Index = psFinder.GetU1Index();
            int u2Index = psFinder.GetU2Index();



            // R(u)の各u微分を計算
            EvaluateDerivativesAtU1(u1Index);
            EvaluateDerivativesAtU2(u2Index);
        }
    }

    private float[] CaluculateArrayU()
    {

        float[] array = new float[N];

        for (int i = 0; i < N; i++)
            array[i] = CalculateU(i);

        return array; 
    }


    private float2[] EvaluateBspline(
        float[,] Nmat,
        int start,
        int end
    )
    {
        int n  = Nmat.GetLength(1);

        float2[] C = new float2[Ns];

        int Nt = Nmat.GetLength(0);
        int idx = 0; // ← C用のインデックス

        for (int j = start; j < end; j++)
        {
            float2 sum = float2.zero;
            for (int i = 0; i < n; i++)
            {
                sum += Nmat[j, i] * controlPoints[i];
            }
            C[idx] = sum;
            idx++;
        }

        return C;
    }

    private float2[] DerivativeBspline(
        float[,] Nmat,
        int start,
        int end
    )
    {
        int n  = Nmat.GetLength(1);

        float2[] C = new float2[Ns];

        int idx = 0; // ← C用のインデックス

        for (int j = start; j < end; j++)
        {
            float2 sum = float2.zero;
            for (int i = 0; i < n; i++)
            {
                sum += Nmat[j, i] * controlPoints[i];
            }
            C[idx] = sum;
            idx++;
        }

        return C;
    }

    private float[] GenerateKnots(
        int N_sampling,
        int p,
        float[] t_ver
    )
    {
        int maxKnots = (N_sampling - 1) + p + 1;
        int numKnotsRoom = maxKnots - 2 * p - 1;

        float endVal = t_ver[t_ver.Length - 1];

        // knot_start, knot_end
        float[] knot = new float[maxKnots + 1];

        // start
        // 最初のp+1個
        for (int i = 0; i <= p; i++)
            knot[i] = 0f;

        // middle
        for (int j = 0; j < numKnotsRoom; j++)
        {
            int j_start = j + 1;
            int j_end   = j + p;

            float sum = 0f;
            for (int i = j_start; i <= j_end; i++)
                sum += t_ver[i];

            knot[p + 1 + j] = sum / p;
        }

        // end
        // 最後ののp+1個
        for (int i = maxKnots - p; i <= maxKnots; i++)
            knot[i] = endVal;

        return knot;
    }

    private float[,] CalculateBsplineBasisMatrix(
        int k, 
        float[] tValue, 
        float[] knots
    )
    {

        int p = k - 1;
        int n = knots.Length - p - 1;

        // 基底関数の数
        int Nt = tValue.Length;

        // 基底関数の行列を初期化
        float[,] Nmat = new float[Nt, n];


        for (int j = 0; j < Nt; j++)
        {
            float x = tValue[j];
            for (int i = 0; i < n; i++)
            {
                Nmat[j, i] = BsplineBasis(i, p, knots, x);
            }
        }
        return Nmat;
    }

    static float BsplineBasis(int i, int p, float[] knots, float x)
    {
        if (p == 0)
        {
            // MATLAB: if k(end-1) > x
            if (knots[knots.Length - 2] > x)
            {
                if(knots[i] <= x && x < knots[i + 1])
                    return 1f;
                else
                    return 0f;
            }
            else
            {
                if(knots[i] <= x && x <= knots[i + 1])
                    return 1f;
                else
                    return 0f;
            }
        }
        else
        {
            float denom1 = knots[i + p] - knots[i];
            float denom2 = knots[i + p + 1] - knots[i + 1];

            float term1, term2;

            if(denom1 == 0f)
                term1 = 0f;
            else
                term1 = (x - knots[i]) / denom1 * BsplineBasis(i, p - 1, knots, x);

            if(denom2 == 0f)
                term2 = 0f;
            else
                term2 = (knots[i + p + 1] - x) / denom2 * BsplineBasis(i + 1, p - 1, knots, x);

            return term1 + term2;
        }
    }

    private float[,] CalculateBsplineBasisDerivativeMatrix(
        int k,
        float[] knots,
        float[] t_sampling,
        int order
    )
    {
        int p = k - 1;
        int n = knots.Length - p - 1;
        int Nt = t_sampling.Length;

        float[,] dNmat = new float[Nt, n];

        for (int j = 0; j < Nt; j++)
        {
            float x = t_sampling[j];

            for (int i = 0; i < n; i++)
            {
                dNmat[j, i] =
                    BsplineBasisDerivative(i, p, knots, x, order);
            }
        }

        return dNmat;
    }

    static float BsplineBasisDerivative(
        int i,
        int p,
        float[] knots,
        float x,
        int order   // 微分階数
    )
    {
        // 0階微分 → 通常の基底関数
        if (order == 0)
            return BsplineBasis(i, p, knots, x);

        // pが0なら導関数は0
        if (p == 0)
            return 0f;

        float denom1 = knots[i + p]     - knots[i];
        float denom2 = knots[i + p + 1] - knots[i + 1];

        float term1 = 0f;
        float term2 = 0f;

        if (denom1 != 0f)
        {
            term1 = p / denom1 * BsplineBasisDeriv(i,     p - 1, knots, x, order - 1);
        }

        if (denom2 != 0f)
        {
            term2 = p / denom2 * BsplineBasisDeriv(i + 1, p - 1, knots, x, order - 1);
        }

        return term1 - term2;
    }

    static float BsplineBasisDeriv(
        int i,
        int p,
        float[] knots,
        float x,
        int order   // 微分階数
    )
    {
        // 0階微分 → 通常の基底関数
        if (order == 0)
            return BsplineBasis(i, p, knots, x);

        // pが0なら導関数は0
        if (p == 0)
            return 0f;

        float denom1 = knots[i + p]     - knots[i];
        float denom2 = knots[i + p + 1] - knots[i + 1];

        float term1 = 0f;
        float term2 = 0f;

        if (denom1 != 0f)
        {
            term1 = p / denom1 * BsplineBasisDeriv(i,     p - 1, knots, x, order - 1);
        }

        if (denom2 != 0f)
        {
            term2 = p / denom2 * BsplineBasisDeriv(i + 1, p - 1, knots, x, order - 1);
        }

        return term1 - term2;
    }

    public int ConvertLocalU1IndexToGlobal(int localIndex)
    {
        // グローバル(0~N)に変換
        int globalIndex = localIndex + frontStart;

        return globalIndex;
    }

    public int ConvertLocalU2IndexToGlobal(int localIndex)
    {
        // グローバル(0~N)に変換
        int globalIndex = localIndex + rearStart;

        return globalIndex;
    }



    public float GetUMin()
    {
        return u_min;
    }

    public float GetUMax()
    {
        return u_max;
    }

    public float[] GetArrayU()
    {
        return u;
    }

    public float CalculateU(int idx)
    {
        // float u = math.lerp(u_min, u_max, ));
        float u = (1 - (float)idx / (N - 1)) * u_min + (float)idx / (N - 1) * u_max;

        return u;
    }

    // idx1はグローバル
    public void EvaluateDerivativesAtU1(int idx1)
    {

        // Debug.Log($"global u1 idx:{idx1}");

        float u1 = CalculateU(idx1);
        int n  = Nmat.GetLength(1);

        // Debug.Log($"n:{n}");

        float2 d1R1du11 = float2.zero;
        float2 d2R1du12 = float2.zero; 
        float2 d3R1du13 = float2.zero; 
        float2 d4R1du14 = float2.zero;

        // 1階微分
        for (int i = 0; i < n; i++)
        {
            d1R1du11 += dbMat1[idx1, i] * controlPoints[i];
        }
        SetD1Rx1du11(d1R1du11.x);
        SetD1Ry1du11(d1R1du11.y);

        // 2階微分
        for (int i = 0; i < n; i++)
        {
            d2R1du12 += dbMat2[idx1, i] * controlPoints[i];
        }
        SetD2Rx1du12(d2R1du12.x);
        SetD2Ry1du12(d2R1du12.y);

        // 3階微分
        for (int i = 0; i < n; i++)
        {
            d3R1du13 += dbMat3[idx1, i] * controlPoints[i];
        }
        SetD3Rx1du13(d3R1du13.x);
        SetD3Ry1du13(d3R1du13.y);

        // 4階微分
        for (int i = 0; i < n; i++)
        {
            d4R1du14 += dbMat4[idx1, i] * controlPoints[i];
        }
        SetD4Rx1du14(d4R1du14.x);
        SetD4Ry1du14(d4R1du14.y);

        // Debug.Log($"d1R1du11:{d1R1du11}, d2R1du12:{d2R1du12}, d3R1du13:{d3R1du13}, d4R1du14:{d4R1du14}");
    }

    public void EvaluateDerivativesAtU2(int idx2)
    {
        float u2 = CalculateU(idx2);
        int n  = Nmat.GetLength(1);

        float2 d1R2du21 = float2.zero; 
        float2 d2R2du22 = float2.zero; 
        float2 d3R2du23 = float2.zero; 
        float2 d4R2du24 = float2.zero;

        // 1階微分
        for (int i = 0; i < n; i++)
        {
            d1R2du21 += dbMat1[idx2, i] * controlPoints[i];
        }
        SetD1Rx2du21(d1R2du21.x);
        SetD1Ry2du21(d1R2du21.y);

        // 2階微分
        for (int i = 0; i < n; i++)
        {
            d2R2du22 += dbMat2[idx2, i] * controlPoints[i];
        }
        SetD2Rx2du22(d2R2du22.x);
        SetD2Ry2du22(d2R2du22.y);

        // 3階微分
        for (int i = 0; i < n; i++)
        {
            d3R2du23 += dbMat3[idx2, i] * controlPoints[i];
        }
        SetD3Rx2du23(d3R2du23.x);
        SetD3Ry2du23(d3R2du23.y);

        // 4階微分
        for (int i = 0; i < n; i++)
        {
            d4R2du24 += dbMat4[idx2, i] * controlPoints[i];
        }
        SetD4Rx2du24(d4R2du24.x);
        SetD4Ry2du24(d4R2du24.y);
    }

    public void CopyFrontBspline(NativeArray<float2> dst)
    {
        if (dst.Length != frontPoints.Length)
        {
            Debug.Log($"dst.Length:{dst.Length} frontPoints.Length:{frontPoints.Length}");
            throw new Exception("Size mismatch");
        }

        dst.CopyFrom(frontPoints);
    }

    public void CopyFrontDerivative1(NativeArray<float2> dst)
    {
        if (dst.Length != frontDerivative1.Length)
            throw new Exception("Size mismatch");

        dst.CopyFrom(frontDerivative1);
    }

    public void CopyRearBspline(NativeArray<float2> dst)
    {
        if (dst.Length != rearPoints.Length)
            throw new Exception("Size mismatch");

        dst.CopyFrom(rearPoints);
    }

    public void CopyRearDerivative1(NativeArray<float2> dst)
    {
        if (dst.Length != rearDerivative1.Length)
            throw new Exception("Size mismatch");

        dst.CopyFrom(rearDerivative1);
    }


    // Setter
    public void SetRx1(float v) { rx1 = v; }
    public void SetRy1(float v) { ry1 = v; }
    public void SetD1Rx1du11(float v) { d1Rx1du11 = v; }
    public void SetD1Ry1du11(float v) { d1Ry1du11 = v; }
    public void SetD2Rx1du12(float v) { d2Rx1du12 = v; }
    public void SetD2Ry1du12(float v) { d2Ry1du12 = v; }
    public void SetD3Rx1du13(float v) { d3Rx1du13 = v; }
    public void SetD3Ry1du13(float v) { d3Ry1du13 = v; }
    public void SetD4Rx1du14(float v) { d4Rx1du14 = v; }
    public void SetD4Ry1du14(float v) { d4Ry1du14 = v; }

    public void SetRx2(float v) { rx2 = v; }
    public void SetRy2(float v) { ry2 = v; }
    public void SetD1Rx2du21(float v) { d1Rx2du21 = v; }
    public void SetD1Ry2du21(float v) { d1Ry2du21 = v; }
    public void SetD2Rx2du22(float v) { d2Rx2du22 = v; }
    public void SetD2Ry2du22(float v) { d2Ry2du22 = v; }
    public void SetD3Rx2du23(float v) { d3Rx2du23 = v; }
    public void SetD3Ry2du23(float v) { d3Ry2du23 = v; }
    public void SetD4Rx2du24(float v) { d4Rx2du24 = v; }
    public void SetD4Ry2du24(float v) { d4Ry2du24 = v; }

    // Getter
    public float GetRx1() => rx1;
    public float GetRy1() => ry1;
    public float GetD1Rx1du11() => d1Rx1du11;
    public float GetD1Ry1du11() => d1Ry1du11;
    public float GetD2Rx1du12() => d2Rx1du12;
    public float GetD2Ry1du12() => d2Ry1du12;
    public float GetD3Rx1du13() => d3Rx1du13;
    public float GetD3Ry1du13() => d3Ry1du13;
    public float GetD4Rx1du14() => d4Rx1du14;
    public float GetD4Ry1du14() => d4Ry1du14;

    public float GetRx2() => rx2;
    public float GetRy2() => ry2;
    public float GetD1Rx2du21() => d1Rx2du21;
    public float GetD1Ry2du21() => d1Ry2du21;
    public float GetD2Rx2du22() => d2Rx2du22;
    public float GetD2Ry2du22() => d2Ry2du22;
    public float GetD3Rx2du23() => d3Rx2du23;
    public float GetD3Ry2du23() => d3Ry2du23;
    public float GetD4Rx2du24() => d4Rx2du24;
    public float GetD4Ry2du24() => d4Ry2du24;

    // public NativeArray<float> GetKnots() => knots;


    public float2[] GetFrontPoints() => frontPoints;
    public float2[] GetRearPoints() => rearPoints;

    public float2[] GetFrontDerivative1() => frontDerivative1;
    public float2[] GetRearDerivative1() => rearDerivative1;

    public int GetNs()
    {
        return Ns;
    }
}