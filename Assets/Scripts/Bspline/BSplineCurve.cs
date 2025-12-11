// using Unity.Mathematics;
// using Unity.Collections;
// using UnityEngine;
// using Unity.Profiling;
// using Unity.Burst;
// using Unity.Jobs;

// public class BSplineCurve
// {

    
//     static ProfilerMarker DeBoorMarker = new ProfilerMarker("BSplineCurve.DeBoor()");
//     static ProfilerMarker FinsSpanMarker = new ProfilerMarker("BSplineCurve.FindSpan()");
//     static ProfilerMarker DerivativePointsMarker = new ProfilerMarker("BSplineCurve.DerivativeAllPoints()");

//     NativeArray<float2> ctrl;     // 制御点 Nd
//     NativeArray<float> knots;     // ノット k
//     NativeArray<float2> points;   // 出力点　N
//     NativeArray<float2> derivative1;   // 出力点　N

//     private BsplineGeometry geo;


//     int k;   // 次数
//     int n;   // 制御点数 - 1 可変
//     int m;   // ノット数　可変
//     int N;   // 出力先のサイズ
//     float u_min; // u_min = knots[3] = 0
//     float u_max; // u_max = knots[19999] = 19996

//     public BSplineCurve(BsplineGeometry bsplineGeometry)
//     {
//         this.geo = bsplineGeometry;
//     }

//     public void Initialize(int Nctrl, int N)
//     {
//         k = 5;               // cubic
//         n = Nctrl - 1; // 997
//         m = n + k + 1; // 1001

//         this.N = N;

//         ctrl = new NativeArray<float2>(Nctrl, Allocator.Persistent);
        
//         knots = new NativeArray<float>(m + 1, Allocator.Persistent);
//         knots = GenerateKnots(knots, k, n ,m);
//         // Nは出力先のサイズ
//         points = new NativeArray<float2>(N, Allocator.Persistent);
//         derivative1 = new NativeArray<float2>(N, Allocator.Persistent);

//         Debug.Log($"n:{n}");


//         u_min = knots[k]; // u_min = knots[3] = 0
//         u_max = knots[n + 1]; // u_max = knots[1001] = 995

//         Debug.Log($"u_min:{u_min}, u_max:{u_max}");
//     }

//     private NativeArray<float> GenerateKnots(NativeArray<float> K, int k, int n, int m)
//     {
//         // 開一様ノット
//         for (int i = 0; i <= k; i++)
//             K[i] = 0;

//         for (int i = k + 1; i <= n; i++)
//             K[i] = i - k;

//         for (int i = n + 1; i < m + 1; i++)
//             K[i] = n - k + 1; // 996

//         // Debug.Log($"K:{K[0]}, {K[1]}, {K[2]}, {K[3]}, {K[4]}, ... , {K[m-5]}, {K[m-4]}, {K[m-3]}, {K[m-2]}, {K[m-1]}");

//         return K;
//     }

//     // 制御点のサイズが可変になるため
//     public NativeArray<float2> Approximate(NativeArray<float2> input)
//     {
//         int Nctrl = input.Length;

//         n = Nctrl - 1;
//         m = n + k + 1;

//         ctrl.CopyFrom(input);

//         u_min = knots[k]; // u_min = knots[3] = 0
//         u_max = knots[n + 1]; // u_max = knots[1001] = 995

//         // points
//         using(GeneratePointsMarker.Auto())
//         {
//             var job = new BSplineJob {
//                 ctrl = ctrl,
//                 knots = knots,
//                 output = points,
//                 k = k,
//                 n = n, 
//                 m = m,
//                 u_min = u_min, 
//                 u_max = u_max
//             };

//             JobHandle handle = job.Schedule(N, 64);
//             handle.Complete();
//         }

//         return points;
//     }

//     public NativeArray<float2> DerivativeAllPoints()
//     {

//         geo.CalculateDerivativeControlPointsJob();


//         // Debug.Log($"u_min ={u_min}, u_max={u_max}");
//         // Debug.Log($"beta1[0]={geo.GetBeta1()[0]}, beta1[degree]={geo.GetBeta1()[k-1]}");
//         // Debug.Log($"knots1[degree]={geo.GetKnots1()[k-1]}, knots1[n+1]={geo.GetKnots1()[n+1]}");


//         using(DerivativePointsMarker.Auto())
//         {
//             var job = new BSplineDerivative1Job {
//                 ctrl = geo.GetBeta1(),
//                 knots = geo.GetKnots(),
//                 degree = k-1,
//                 u_min = u_min, // 0
//                 u_max = u_max, // 996
//                 results = derivative1
//             };

//             JobHandle handle = job.Schedule(N, 64);
//             handle.Complete();
//         }

//         return derivative1;
//     }

//     public void Dispose()
//     {
//         if (ctrl.IsCreated) ctrl.Dispose();
//         if (knots.IsCreated) knots.Dispose();
//         if (points.IsCreated) points.Dispose();
//         if (derivative1.IsCreated) derivative1.Dispose();

//     }

//     public float2[] GetPointsManaged()
//     {
//         float2[] arr = new float2[N];
//         for (int i = 0; i < N; i++)
//             arr[i] = points[i];
//         return arr;
//     }

//     public int GetN()
//     {
//         return N;
//     }
// }

 