#include "curve.h"
#include "vertexrecorder.h"
using namespace std;

const float c_pi = 3.14159265358979323846f;

namespace
{
// Approximately equal to.  We don't want to use == because of
// precision issues with floating point.
inline bool approx(const Vector3f& lhs, const Vector3f& rhs)
{
	const float eps = 1e-8f;
	return (lhs - rhs).absSquared() < eps;
}


}

#if 0
Curve evalBezier(const vector< Vector3f >& P, unsigned steps)
{
	// Check
	if (P.size() < 4 || P.size() % 3 != 1)
	{
		cerr << "evalBezier must be called with 3n+1 control points." << endl;
		exit(0);
	}

	// TODO:
	// You should implement this function so that it returns a Curve
	// (e.g., a vector< CurvePoint >).  The variable "steps" tells you
	// the number of points to generate on each piece of the spline.
	// At least, that's how the sample solution is implemented and how
	// the SWP files are written.  But you are free to interpret this
	// variable however you want, so long as you can control the
	// "resolution" of the discretized spline curve with it.

	// Make sure that this function computes all the appropriate
	// Vector3fs for each CurvePoint: V,T,N,B.
	// [NBT] should be unit and orthogonal.

	// Also note that you may assume that all Bezier curves that you
	// receive have G1 continuity.  Otherwise, the TNB will not be
	// be defined at points where this does not hold.

	cerr << "\t>>> evalBezier has been called with the following input:" << endl;

	cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
	for (int i = 0; i < (int)P.size(); ++i)
	{
		cerr << "\t>>> " << P[i] << endl;
	}

	cerr << "\t>>> Steps (type steps): " << steps << endl;
	cerr << "\t>>> Returning empty curve." << endl;

	// Right now this will just return this empty curve.
	return Curve();
}
#endif

Curve evalBezier(const vector< Vector3f >& P, unsigned steps)
{
    // 检查控制点数量是否合法
    if (P.size() < 4 || P.size() % 3 != 1)
    {
        cerr << "evalBezier must be called with 3n+1 control points." << endl;
        exit(0);
    }

    // 输出调试信息
    cerr << "\t>>> evalBezier has been called with the following input:" << endl;
    cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
    for (int i = 0; i < (int)P.size(); ++i)
    {
        cerr << "\t>>> " << P[i] << endl;
    }
    cerr << "\t>>> Steps (type steps): " << steps << endl;

    // 初始化返回的曲线
    Curve curve;

    // 分段处理贝塞尔曲线
    for (size_t i = 0; i + 3 < P.size(); i += 3)
    {
        // 获取当前段的 4 个控制点
        Vector3f P0 = P[i];
        Vector3f P1 = P[i + 1];
        Vector3f P2 = P[i + 2];
        Vector3f P3 = P[i + 3];

        // 生成当前段的曲线点
        for (unsigned j = 0; j <= steps; ++j)
        {
            // 计算参数 t
            float t = static_cast<float>(j) / steps;

            // 计算贝塞尔曲线的位置
            Vector3f position = pow(1 - t, 3) * P0 +
                                3 * pow(1 - t, 2) * t * P1 +
                                3 * (1 - t) * pow(t, 2) * P2 +
                                pow(t, 3) * P3;

            // 计算贝塞尔曲线的切向量
            Vector3f tangent = 3 * pow(1 - t, 2) * (P1 - P0) +
                               6 * (1 - t) * t * (P2 - P1) +
                               3 * pow(t, 2) * (P3 - P2);
            tangent.normalize();

            // 计算法向量和副法向量
            Vector3f normal = Vector3f::cross(tangent, Vector3f(0, 1, 0));
            normal.normalize();
            Vector3f binormal = Vector3f::cross(tangent, normal);
            binormal.normalize();

            // 创建曲线点并添加到曲线中
            CurvePoint point;
            point.V = position;
            point.T = tangent;
            point.N = normal;
            point.B = binormal;
            curve.push_back(point);
        }
    }

    cerr << "\t>>> Returning curve with " << curve.size() << " points." << endl;
    return curve;
}

#if 0
Curve evalBspline(const vector< Vector3f >& P, unsigned steps)
{
	// Check
	if (P.size() < 4)
	{
		cerr << "evalBspline must be called with 4 or more control points." << endl;
		exit(0);
	}

	// TODO:
	// It is suggested that you implement this function by changing
	// basis from B-spline to Bezier.  That way, you can just call
	// your evalBezier function.

	cerr << "\t>>> evalBSpline has been called with the following input:" << endl;

	cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
	for (int i = 0; i < (int)P.size(); ++i)
	{
		cerr << "\t>>> " << P[i] << endl;
	}

	cerr << "\t>>> Steps (type steps): " << steps << endl;
	cerr << "\t>>> Returning empty curve." << endl;

	// Return an empty curve right now.
	return Curve();
}
#endif

Curve evalBspline(const vector< Vector3f >& P, unsigned steps)
{
    // 检查控制点数量是否合法
    if (P.size() < 4)
    {
        cerr << "evalBspline must be called with 4 or more control points." << endl;
        exit(0);
    }

    // 输出调试信息
    cerr << "\t>>> evalBSpline has been called with the following input:" << endl;
    cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
    for (int i = 0; i < (int)P.size(); ++i)
    {
        cerr << "\t>>> " << P[i] << endl;
    }
    cerr << "\t>>> Steps (type steps): " << steps << endl;

    // 初始化返回的曲线
    Curve curve;

    // 将 B 样条曲线转换为分段贝塞尔曲线
    for (size_t i = 0; i + 3 < P.size(); ++i)
    {
        // 获取当前段的 4 个控制点
        Vector3f P0 = P[i];
        Vector3f P1 = P[i + 1];
        Vector3f P2 = P[i + 2];
        Vector3f P3 = P[i + 3];

        // 将 B 样条控制点转换为贝塞尔控制点
        Vector3f B0 = (P0 + 4 * P1 + P2) / 6;
        Vector3f B1 = (2 * P1 + P2) / 3;
        Vector3f B2 = (P1 + 2 * P2) / 3;
        Vector3f B3 = (P1 + 4 * P2 + P3) / 6;

        // 构建贝塞尔控制点向量
        vector< Vector3f > bezierPoints = { B0, B1, B2, B3 };

        // 调用 evalBezier 生成贝塞尔曲线段
        Curve bezierSegment = evalBezier(bezierPoints, steps);

        // 将生成的贝塞尔曲线段添加到结果曲线中
        curve.insert(curve.end(), bezierSegment.begin(), bezierSegment.end());
    }

    cerr << "\t>>> Returning curve with " << curve.size() << " points." << endl;
    return curve;
}

Curve evalCircle(float radius, unsigned steps)
{
	// This is a sample function on how to properly initialize a Curve
	// (which is a vector< CurvePoint >).

	// Preallocate a curve with steps+1 CurvePoints
	Curve R(steps + 1);

	// Fill it in counterclockwise
	for (unsigned i = 0; i <= steps; ++i)
	{
		// step from 0 to 2pi
		float t = 2.0f * c_pi * float(i) / steps;

		// Initialize position
		// We're pivoting counterclockwise around the y-axis
		R[i].V = radius * Vector3f(cos(t), sin(t), 0);

		// Tangent vector is first derivative
		R[i].T = Vector3f(-sin(t), cos(t), 0);

		// Normal vector is second derivative
		R[i].N = Vector3f(-cos(t), -sin(t), 0);

		// Finally, binormal is facing up.
		R[i].B = Vector3f(0, 0, 1);
	}

	return R;
}

void recordCurve(const Curve& curve, VertexRecorder* recorder)
{
	const Vector3f WHITE(1, 1, 1);
	for (int i = 0; i < (int)curve.size() - 1; ++i)
	{
		recorder->record_poscolor(curve[i].V, WHITE);
		recorder->record_poscolor(curve[i + 1].V, WHITE);
	}
}
void recordCurveFrames(const Curve& curve, VertexRecorder* recorder, float framesize)
{
	Matrix4f T;
	const Vector3f RED(1, 0, 0);
	const Vector3f GREEN(0, 1, 0);
	const Vector3f BLUE(0, 0, 1);
	
	const Vector4f ORGN(0, 0, 0, 1);
	const Vector4f AXISX(framesize, 0, 0, 1);
	const Vector4f AXISY(0, framesize, 0, 1);
	const Vector4f AXISZ(0, 0, framesize, 1);

	for (int i = 0; i < (int)curve.size(); ++i)
	{
		T.setCol(0, Vector4f(curve[i].N, 0));
		T.setCol(1, Vector4f(curve[i].B, 0));
		T.setCol(2, Vector4f(curve[i].T, 0));
		T.setCol(3, Vector4f(curve[i].V, 1));
 
		// Transform orthogonal frames into model space
		Vector4f MORGN  = T * ORGN;
		Vector4f MAXISX = T * AXISX;
		Vector4f MAXISY = T * AXISY;
		Vector4f MAXISZ = T * AXISZ;

		// Record in model space
		recorder->record_poscolor(MORGN.xyz(), RED);
		recorder->record_poscolor(MAXISX.xyz(), RED);

		recorder->record_poscolor(MORGN.xyz(), GREEN);
		recorder->record_poscolor(MAXISY.xyz(), GREEN);

		recorder->record_poscolor(MORGN.xyz(), BLUE);
		recorder->record_poscolor(MAXISZ.xyz(), BLUE);
	}
}

