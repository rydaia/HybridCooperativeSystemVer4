using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Unity.Profiling;


public class PsFinder
{
    static ProfilerMarker PsFinderMarker = new ProfilerMarker("PsFinder.StepPsFinder()");

    private TrajectoryGenerator trajectoryGenerator;
    private VehicleRobotState vehicleRobotState; 
    private BsplineGeometry bsplineGeometry; 
    private VehicleParameters vehicleParams;

    // 配列 ベジェ点と1階微分の前計算 （q = i/(N-1)）
    NativeArray<float2> R;   // (x,z)
    NativeArray<float2> dR;  // dR/dq
    NativeArray<float> scoreArray;

    public bool isInitialSeachedPs1;
    public bool isInitialSeachedPs2;

    public int u1Index, u2Index;
    public int prevPs1Index, prevPs2Index;

    private int N;
    private float epsilon = 0.001f; // Ps探索における内積の判定


    public PsFinder(
        VehicleRobotState robot, 
        TrajectoryGenerator trajectory,
        BsplineGeometry geo,
        VehicleParameters parms
    )
    {
        this.trajectoryGenerator = trajectory;
        this.vehicleRobotState = robot;
        this.bsplineGeometry = geo;
        this.vehicleParams = parms;

    }

    public void Initialize()
    {
        isInitialSeachedPs1 = false;
        isInitialSeachedPs2 = false;

        N = trajectoryGenerator.GetN();

        R   = new NativeArray<float2>(N, Allocator.Persistent);
        dR  = new NativeArray<float2>(N, Allocator.Persistent);

        scoreArray = new NativeArray<float>(N, Allocator.Persistent);
    }


    public void StepPsFinder()
    {
        using(PsFinderMarker.Auto())
        {
            prevPs1Index = u1Index;
            prevPs2Index = u2Index;

            RebuildCache();
            // Ps1
            if (!isInitialSeachedPs1)
            {
                u1Index = FindPs1PointGlobally_Job();
            }
            else
            {
                // u1Index = FindPs1PointLocally_Job(100);
                u1Index = FindPs1PointGlobally_Job();
            }


            // ここでrx1, ry1をセット
            bsplineGeometry.SetRx1(trajectoryGenerator.bsplineNative[u1Index].x);
            bsplineGeometry.SetRy1(trajectoryGenerator.bsplineNative[u1Index].y);

  

            // Ps2
            if (!isInitialSeachedPs2)
            {
                u2Index = FindPs2PointGlobally_Job();
            }
            else
            {
                u2Index = FindPs2PointLocally_Job(200);
            }

            // Debug.Log($"u1Index:{u1Index}, u2Index:{u2Index}, prevPs1Index:{prevPs1Index}, prevPs2Index:{prevPs2Index}");

        }
    }


    public void RebuildCache() // 経路が変わる時だけ呼ぶ
    {
        if (!R.IsCreated) return;

        R = trajectoryGenerator.GetBsplineNative();
        dR = trajectoryGenerator.GetBsplineDerivativeNative();
    }

    public void Dispose()
    {
        if (R.IsCreated)   R.Dispose();
        if (dR.IsCreated)  dR.Dispose();
        if (scoreArray.IsCreated) scoreArray.Dispose();
    }

    // ========= Ps1探索 =========
    // 全域探索
    public int FindPs1PointGlobally_Job()
    {
        // Debug.Log($"N:{N}");
        // Debug.Log($"epsilon:{epsilon}");


        if (N <= 1) return 0;

        // Debug.Log($"R[0].x, R[0].y:{R[0].x}, {R[0].y}");
        // Debug.Log($"dR[0].x, dR[0].y:{dR[0].x}, {dR[0].y}");
        // Debug.Log($"dR[1].x, dR[1].y:{dR[1].x}, {dR[1].y}");
        // Debug.Log($"vehicleRobotState.GetX1(), vehicleRobotState.GetY1():{vehicleRobotState.GetX1()}, {vehicleRobotState.GetY1()}");

        // ここでIJobParallelForの呼び出し 各ループを並列に処理する
        var job = new Ps1EvalJob
        {
            posCurr = new float2(vehicleRobotState.GetX1(), vehicleRobotState.GetY1()),
            nBspline = N,
            startIdx = 0,
            endIdx = N - 1,
            epsilon = epsilon,
            R = R,
            dR = dR,
            score = scoreArray
        };

        // 128??
        JobHandle h = job.Schedule(N, 128);
        h.Complete();


        // Debug.Log($"scoreArray[0],scoreArray[5000]:{scoreArray[0]}, {scoreArray[5000]}");


        // 単スレ reduction
        float best = float.PositiveInfinity;
        int bestIdx = prevPs1Index;
        for (int i = 0; i < N; i++)
        {
            // Debug.Log($"i:{i}, scoreArray[i]:{scoreArray[i]}");
            // Debug.Log($"i:{i}, R[i].x, R[i].y:{R[i].x}, {R[i].y}");


            float s = scoreArray[i];
            if (s < best) { best = s; bestIdx = i; }
        }

        isInitialSeachedPs1 = true;
        return bestIdx;
    }

    // 局所探索
    public int FindPs1PointLocally_Job(int range = 100)
    {
        if (N <= 1) return 0;

        int start = Mathf.Max(0, prevPs1Index - range);
        int end   = Mathf.Min(N-1, prevPs1Index + range);

        // --- 終端近くでは必ず最後まで探索する ---
        if (prevPs1Index + range >= N - 1)
            end = N - 1;

        // ここでIJobParallelForの呼び出し 各ループを並列に処理する
        var job = new Ps1EvalJob
        {
            posCurr = new float2(vehicleRobotState.GetX1(), vehicleRobotState.GetY1()),
            nBspline = N,
            startIdx = start,
            endIdx = end,
            epsilon = epsilon,
            R = R,
            dR = dR,
            score = scoreArray
        };

        JobHandle h = job.Schedule(N, 128);
        h.Complete();

        float best = float.PositiveInfinity;
        int bestIdx = prevPs1Index;
        for (int i = start; i <= end; i++)
        {
            float s = scoreArray[i];
            if (s < best) 
            { 
                best = s; 
                bestIdx = i; 
            }
        }

        isInitialSeachedPs1 = true;
        return bestIdx;
    }

    // ========= Ps2 =========
    // 全域探索
    public int FindPs2PointGlobally_Job()
    {
        if (N <= 1) 
        { 
            prevPs2Index = 0; 
            return 0; 
        }

        float2 _ps1 = new float2(bsplineGeometry.GetRx1(), bsplineGeometry.GetRy1());
        float L2abs = math.abs(vehicleParams.GetL2());
        float r2 = L2abs * L2abs;

        // Debug.Log($"_ps1:{_ps1}");
        // Debug.Log($"L2:{vehicleParams.GetL2()}");

        int start = u1Index; // ここから後ろ（小さいq）へ走査していく
        int end   = 0;

        // jpbの呼び出し
        var job = new Ps2EvalJob
        {
            ps1 = _ps1,
            r2 = r2,
            nBspline = N,
            startIdx = start,
            endIdx = end,
            R = R,
            err = scoreArray
        };

        JobHandle h = job.Schedule(N, 256);
        h.Complete();

        float bestErr = float.PositiveInfinity;
        int bestIdx = start;
        for (int i = start; i >= end; --i)
        {
            float e = scoreArray[i];
            if (e < bestErr) 
            { 
                bestErr = e; 
                bestIdx = i; 
            }
        }

        isInitialSeachedPs2 = true;
        Debug.Log($"global ps2 seach bestId:{bestIdx}");

        return bestIdx;

    }

    // 局所探索
    public int FindPs2PointLocally_Job(int range = 200)
    {
        if (N <= 1) 
        { 
            prevPs2Index = 0; 
            return 0; 
        }

        float2 ps1  = new float2(bsplineGeometry.GetRx1(), bsplineGeometry.GetRy1());
        float L2abs = math.abs(vehicleParams.GetL2());
        float r2 = L2abs * L2abs;

        int start = Mathf.Min(N - 1, prevPs2Index + range);
        int end   = Mathf.Max(0,     prevPs2Index - range);

        // job呼び出し
        var job = new Ps2EvalJob
        {
            ps1 = ps1,
            r2 = r2,
            nBspline = N,
            startIdx = start, // こちらは大きい→小さい方向に使う
            endIdx = end,
            R = R,
            err = scoreArray
        };

        JobHandle h = job.Schedule(N, 256);
        h.Complete();

        float bestErr = float.PositiveInfinity;
        int bestIdx = start;
        for (int i = start; i >= end; --i)
        {
            float e = scoreArray[i];
            if (e < bestErr) 
            { 
                bestErr = e; 
                bestIdx = i; 
            }
        }

        isInitialSeachedPs2 = true;
        return bestIdx;
    }

    public int GetU1Index() => u1Index;
    public int GetU2Index() => u2Index;

}

