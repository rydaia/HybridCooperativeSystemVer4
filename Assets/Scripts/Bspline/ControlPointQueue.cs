using UnityEngine;
using Unity.Profiling;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;

public class ControlPointQueue
{

    // プロファイルマーカー
    static ProfilerMarker pastJobMarker = new ProfilerMarker("CPQueue.PastJob");
    static ProfilerMarker futureJobMarker = new ProfilerMarker("CPQueue.FutureJob");
    static ProfilerMarker mergeJobMarker = new ProfilerMarker("CPQueue.MergeJob");

    // 過去
    NativeArray<float2> past;      // 過去キュー（リングバッファ）
    int pastHead;                  // FIFO head index（取り出し位置）
    int pastCount;                 // 現在の要素数（np 固定）
    int Np;

    // 未来
    NativeArray<float2> future;    // 未来予測 (Nf)
    int Nf;

    private float dsTotal = 0f;
    private float dt;      // シミュレーション刻み幅

    private TargetPointState target;
    private TargetPointCtrl TPctrl;
    private TrajectoryGenerator trajectory;

    private float prevTotalS;

    // コントラスタ
    public ControlPointQueue()
    {
    }

    public void Initialize(float dt, CalculationManager calc)
    {

        target = calc.targetPointState;
        TPctrl = calc.targetPointCtrl;
        trajectory = calc.trajectoryGenerator;

        this.Np = trajectory.GetNp();
        this.Nf = trajectory.GetNf();
        this.dt = dt;

        past     = new NativeArray<float2>(Np, Allocator.Persistent);
        future   = new NativeArray<float2>(Nf, Allocator.Persistent);

        pastHead = 0;
        pastCount = 0;

        // / --- 初回の過去・未来データ生成 ---
        Vector2 Tp0 = target.GetPosition();
        float theta0 = target.getTheta();
        float kappa0 = calc.targetPointState.getKappa();

        InitializePastQueue(Tp0, theta0);
        InitializeFuturePoints(Tp0, theta0, kappa0);

        Debug.Log($"InitializePastData complete. Length={past.Length}");

    }

    public void Update( Vector2 Tp, float theta, float v1, float kappa )
    {

        // 目標点が止まっている時は過去の軌跡データの更新はしない
        using (pastJobMarker.Auto())
        {
            if(v1 > 0.0f) UpdatePastQueue(Tp, v1);
        }

        using (futureJobMarker.Auto())
        {
            UpdateFuturePoints(Tp, theta, kappa);
        }
    }

    // TPの初期値からnp分姿勢角の荷台方向へds刻みで格納していく
    // 過去 NP 個
    // 初期位置TP から後方へ ds 刻みで並べる
    // 逆にする
    public void InitializePastQueue(Vector2 Tp, float theta)
    {
        pastHead = 0;
        pastCount = 0;

        // 過去データの初期化時は距離0.01間隔
        float dp = 0.01f;

        Debug.Log($"Tp={Tp}");


        float2 d = new float2(math.cos(theta), math.sin(theta));

        float2 start = new float2(
            Tp.x - d.x * dt * (Np - 1),
            Tp.y - d.y * dt * (Np - 1)
        );

        Debug.Log($"start={start}");


        for (int i = 0; i < Np; i++)
        {
            float2 p = start + d * (dp * i);
            // Debug.Log($"p={p}");
            EnqueuePast(p);
        }

        Debug.Log($"InitializePastQueue complete. Count={pastCount}");
    }

    void UpdatePastQueue(Vector2 Tp, float v1)
    {
        float ds = 0.01f;

        // 前回追加した時の点
        float2 last = past[(pastHead + pastCount - 1) % Np];
        // 現在の点
        float2 curr = new float2(Tp.x, Tp.y);

        float2 diff = curr - last;
        float dist = math.length(diff);

        if (dist < ds)
            return;

        float2 dir = diff / dist;

        // 追加すべき点の数
        int num = (int)(dist / ds);

        for (int i = 1; i <= num; i++)
        {
            float2 p = last + dir * (ds * i);
            EnqueuePast(p);
        }
    }

    // NativeArray FIFO (enqueue)
    void EnqueuePast(float2 v)
    {
        if (pastCount < Np)
        {
            int tail = (pastHead + pastCount) % Np;
            past[tail] = v;
            pastCount++;
        }
        else
        {
            // 上書き（最大サイズ時）
            past[pastHead] = v;
            pastHead = (pastHead + 1) % Np;
        }
    }

    // dequeue n 個
    void DequeuePast(int n)
    {
        n = math.min(n, pastCount);
        pastHead = (pastHead + n) % Np;
        pastCount -= n;
    }


    // TPの初期値からnf分姿勢角(theta)方向へへds刻みで格納していく
    // 初期位置TP から未来方向 θ に向かってds 間隔で NF 個さらに曲率 κ に沿って θ を更新しながら未来点を生成する
    public void InitializeFuturePoints(Vector2 Tp, float theta, float kappa)
    {
        float2 tp = new float2(Tp.x, Tp.y);

        Debug.Log($"kappa:{kappa}");


        // 未来の軌道データは距離0.01間隔で初期化
        float df = 0.01f;

        for (int k = 0; k < Nf; k++)
        {
            float s = df * (k + 1);
            float theta_k = theta + kappa * s;

            future[k] = new float2(
                tp.x + math.cos(theta_k) * s,
                tp.y + math.sin(theta_k) * s
            );
        }
    }

    void UpdateFuturePoints(Vector2 Tp, float theta, float kappa)
    {
        // 4. 未来予測データを再生成
        var job = new FutureJob
        {
            Tp = new float2(Tp.x, Tp.y),
            theta = theta,
            kappa = kappa,
            d = dt,
            output = future
        };

        JobHandle handle = job.Schedule(Nf, 64);
        handle.Complete();
    }

    public void Dispose()
    {
        if (past.IsCreated) past.Dispose();
        if (future.IsCreated) future.Dispose();
    }

    public int GetNp()
    {
        return Np;
    }

    public NativeArray<float2> GetFutureNative()
    {
        return future;
    }

    public NativeArray<float2> GetPastNative()
    {
        NativeArray<float2> arr = new NativeArray<float2>(Np, Allocator.Persistent);

        for (int i = 0; i < pastCount; i++)
        {
            int idx = (pastHead + i) % Np;
            arr[i] = past[idx];
        }
        return arr;
    }

    public float2[] GetPastPointsManaged()
    {
        float2[] arr = new float2[pastCount];
        for (int i = 0; i < pastCount; i++)
        {
            int idx = (pastHead + i) % Np;
            arr[i] = past[idx];
        }
        return arr;
    }

}
