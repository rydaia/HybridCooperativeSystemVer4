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
    private SimulationManager sim;

    // 配列 ベジェ点と1階微分の前計算 （q = i/(N-1)）
    private NativeArray<float2> frontBspline;   // (x,z)
    private NativeArray<float2> rearBspline;   // (x,z)
    NativeArray<float2> dFrontBspline;  // dR/dq
    NativeArray<float2> dRearBspline;  // dR/dq

    NativeArray<float> scoreArray;

    public bool isInitialSeachedPs1;
    public bool isInitialSeachedPs2;

    public int localU1Index, localU2Index;
    public int globalU1Index, globalU2Index;

    public int prevLocalPs1Index, prevLocalPs2Index;

    public float u1, u2;

    private int N;
    private int Ns;

    private float epsilon = 0.001f; // Ps探索における内積の判定


    public PsFinder(
        VehicleRobotState robot, 
        TrajectoryGenerator trajectory,
        VehicleParameters parms,
        SimulationManager sim
    )
    {
        this.trajectoryGenerator = trajectory;
        this.vehicleRobotState = robot;
        this.vehicleParams = parms;

        this.sim = sim;
    }

    public void Initialize(BsplineGeometry geo)
    {

        this.bsplineGeometry = geo;

        isInitialSeachedPs1 = false;
        isInitialSeachedPs2 = false;

        localU1Index = 0;
        localU2Index = 0;

        globalU1Index = 0;
        globalU2Index = 0;

        N = trajectoryGenerator.GetN();
        Ns = bsplineGeometry.GetNs();

        frontBspline   = new NativeArray<float2>(Ns, Allocator.Persistent);
        rearBspline   = new NativeArray<float2>(Ns, Allocator.Persistent);

        dFrontBspline = new NativeArray<float2>(Ns, Allocator.Persistent);
        dRearBspline = new NativeArray<float2>(Ns, Allocator.Persistent);

        scoreArray = new NativeArray<float>(Ns, Allocator.Persistent);
    }


    public void StepPsFinder()
    {
        using(PsFinderMarker.Auto())
        {
            prevLocalPs1Index = localU1Index;
            prevLocalPs2Index = localU2Index;

            if(trajectoryGenerator.GetIsUpdateCPFlag())
            {
                RebuildCache();
            }

            if (!isInitialSeachedPs1)
            {
                localU1Index = FindPs1PointGlobally_Job();
            }
            else
            {
                // localU1Index = FindPs1PointLocally_Job(100);
                localU1Index = FindPs1PointGlobally_Job();
            }


            if (localU1Index >= Ns)
            {
                Debug.LogError($"local u1Index out of front range: {localU1Index}");
                sim.StopSimulation();
            }
            
            if (localU1Index == 0)
            {
                Debug.LogError($"local u1Index out of front range: {localU1Index}");
                sim.StopSimulation();
            }
            

            globalU1Index = bsplineGeometry.ConvertLocalU1IndexToGlobal(localU1Index);

            if (globalU1Index >= N)
                Debug.LogError($"global u1Index out of front range: {globalU1Index}");

            u1 = bsplineGeometry.CalculateU(globalU1Index);

            // Debug.Log($"localU1Index:{localU1Index}, globalU1Index:{globalU1Index}");

            // ここでrx1, ry1をセット
            bsplineGeometry.SetRx1(bsplineGeometry.frontPoints[localU1Index].x);
            bsplineGeometry.SetRy1(bsplineGeometry.frontPoints[localU1Index].y);

            // Ps2
            if (!isInitialSeachedPs2)
            {
                localU2Index = FindPs2PointGlobally_Job();
            }
            else
            {
                localU2Index = FindPs2PointGlobally_Job();
                // localU2Index = FindPs2PointLocally_Job(200);
            }

            if (localU2Index >= Ns)
            {
                Debug.LogError($"PsFinde Failure: local u2Index out of front range: {localU2Index}");
                sim.StopSimulation();
            }



            if (localU2Index == 0)
            {
                Debug.LogError($"PsFinde Failure: local u2Index out of front range: {localU2Index}");
                sim.StopSimulation();
            }

            globalU2Index = bsplineGeometry.ConvertLocalU2IndexToGlobal(localU2Index);

            if (globalU2Index >= N)
                Debug.LogError($"global u2Index out of front range: {globalU2Index}");

            u2 = bsplineGeometry.CalculateU(globalU2Index);

            // R(u2)
            bsplineGeometry.SetRx2(bsplineGeometry.rearPoints[localU2Index].x);
            bsplineGeometry.SetRy2(bsplineGeometry.rearPoints[localU2Index].y);
            // Debug.Log($"u1Index:{u1Index}, u2Index:{u2Index}, prevPs1Index:{prevPs1Index}, prevPs2Index:{prevPs2Index}");

        }
    }

    public void RebuildCache()
    {
        bsplineGeometry.CopyFrontBspline(frontBspline);
        bsplineGeometry.CopyFrontDerivative1(dFrontBspline);

        bsplineGeometry.CopyRearBspline(rearBspline);
        bsplineGeometry.CopyRearDerivative1(dRearBspline);
    }



    // public void RebuildCache() // 経路が変わる時だけ呼ぶ
    // {
    //     if (!frontBspline.IsCreated) return;
    //     if (!rearBspline.IsCreated) return;

    //     frontBspline = bsplineGeometry.GetFrontBsplineNative();
    //     rearBspline = bsplineGeometry.GetRearBsplineNative();

    //     dFrontBspline = bsplineGeometry.GetFrontDerivative1Native();
    //     dRearBspline = bsplineGeometry.GetRearDerivative1Native();
    // }

    public void Dispose()
    {
        if (frontBspline.IsCreated)   frontBspline.Dispose();
        if (dFrontBspline.IsCreated)  dFrontBspline.Dispose();

        if (rearBspline.IsCreated)   rearBspline.Dispose();
        if (dRearBspline.IsCreated)  dRearBspline.Dispose();

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
            nBspline = Ns, //
            startIdx = 0,
            endIdx = Ns - 1,
            epsilon = epsilon,
            R = frontBspline,
            dR = dFrontBspline,
            score = scoreArray
        };

        // 128??
        JobHandle h = job.Schedule(Ns, 128);
        h.Complete();


        // Debug.Log($"scoreArray[0],scoreArray[5000]:{scoreArray[0]}, {scoreArray[5000]}");


        // 単スレ reduction
        float best = float.PositiveInfinity;
        int bestIdx = prevLocalPs1Index;
        for (int i = 0; i < Ns; i++)
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
        if (Ns <= 1) return 0;

        int start = Mathf.Max(0, prevLocalPs1Index - range);
        int end   = Mathf.Min(N-1, prevLocalPs1Index + range);

        // --- 終端近くでは必ず最後まで探索する ---
        if (prevLocalPs1Index + range >= Ns - 1)
            end = Ns - 1;

        // ここでIJobParallelForの呼び出し 各ループを並列に処理する
        var job = new Ps1EvalJob
        {
            posCurr = new float2(vehicleRobotState.GetX1(), vehicleRobotState.GetY1()),
            nBspline = Ns,
            startIdx = start,
            endIdx = end,
            epsilon = epsilon,
            R = frontBspline,
            dR = dFrontBspline,
            score = scoreArray
        };

        JobHandle h = job.Schedule(Ns, 128);
        h.Complete();

        float best = float.PositiveInfinity;
        int bestIdx = prevLocalPs1Index;
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
        if (Ns <= 1) 
        { 
            prevLocalPs2Index = 0; 
            return 0; 
        }

        float2 _ps1 = new float2(bsplineGeometry.GetRx1(), bsplineGeometry.GetRy1());
        float L2abs = math.abs(vehicleParams.GetL2());
        float r2 = L2abs * L2abs;

        // Debug.Log($"_ps1:{_ps1}");
        // Debug.Log($"L2:{vehicleParams.GetL2()}");

        int start = Ns-1; // ここから後ろ（小さいq）へ走査していく
        int end   = 0;

        // jpbの呼び出し
        var job = new Ps2EvalJob
        {
            ps1 = _ps1,
            r2 = r2,
            nBspline = Ns,
            startIdx = start,
            endIdx = end,
            R = rearBspline,
            err = scoreArray
        };

        JobHandle h = job.Schedule(Ns, 256);
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
        // Debug.Log($"global ps2 seach bestId:{bestIdx}");

        return bestIdx;
    }

    // 局所探索
    public int FindPs2PointLocally_Job(int range = 200)
    {
        if (Ns <= 1) 
        { 
            prevLocalPs2Index = 0; 
            return 0; 
        }

        float2 ps1  = new float2(bsplineGeometry.GetRx1(), bsplineGeometry.GetRy1());
        float L2abs = math.abs(vehicleParams.GetL2());
        float r2 = L2abs * L2abs;

        int start = Mathf.Min(Ns - 1, prevLocalPs2Index + range);
        int end   = Mathf.Max(0,     prevLocalPs2Index - range);

        // job呼び出し
        var job = new Ps2EvalJob
        {
            ps1 = ps1,
            r2 = r2,
            nBspline = Ns,
            startIdx = start, // こちらは大きい→小さい方向に使う
            endIdx = end,
            R = rearBspline,
            err = scoreArray
        };

        JobHandle h = job.Schedule(Ns, 256);
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

    public int GetU1Index() => globalU1Index;
    public int GetU2Index() => globalU2Index;

    public float GetU1() => u1;
    public float GetU2() => u2;
}

