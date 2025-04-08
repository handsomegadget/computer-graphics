#include "surf.h"
#include "vertexrecorder.h"
#include <assert.h>
using namespace std;

namespace
{
    
    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        for (unsigned i=0; i<profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;
    
        return true;
    }
}

// DEBUG HELPER
Surface quad() { 
	Surface ret;
	ret.VV.push_back(Vector3f(-1, -1, 0));
	ret.VV.push_back(Vector3f(+1, -1, 0));
	ret.VV.push_back(Vector3f(+1, +1, 0));
	ret.VV.push_back(Vector3f(-1, +1, 0));

	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));

	ret.VF.push_back(Tup3u(0, 1, 2));
	ret.VF.push_back(Tup3u(0, 2, 3));
	return ret;
}
#if 0
Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
	surface = quad();
    
    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // TODO: Here you should build the surface.  See surf.h for details.

    cerr << "\t>>> makeSurfRev called (but not implemented).\n\t>>> Returning empty surface." << endl;
 
    return surface;
}

#endif
Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;

    // 检查曲线是否在 xy 平面上
    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // 初始化表面数据
    surface.VV.clear();
    surface.VN.clear();
    surface.VF.clear();

    // 遍历旋转步数
    for (unsigned i = 0; i < steps; ++i)
    {
        float theta = (2 * M_PI * i) / steps;  // 当前旋转角度
        
        // 构造旋转矩阵
        Matrix4f R = Matrix4f(cos(theta),  0, sin(theta), 0,
             0,           1, 0, 0,
            -sin(theta),  0, cos(theta), 0,
        0 , 0, 0, 1);

        // 计算逆转置矩阵（对于旋转矩阵，逆转置 = 原矩阵）
        Matrix3f invTransR = R.getSubmatrix3x3(0,0);  // 实际上等于R本身

        // 遍历曲线点
        for (size_t j = 0; j < profile.size(); ++j)
        {
            // 顶点旋转
            Vector4f point = Vector4f(profile[j].V, 1);
            Vector4f rotatedPoint = R * point;  // 应用旋转矩阵
            surface.VV.push_back(rotatedPoint.xyz());

            // 法向量变换（使用逆转置矩阵）
            Vector3f originalNormal = profile[j].N;
            Vector3f transformedNormal = invTransR * originalNormal;
            
            transformedNormal.normalize();  // 确保单位长度
            transformedNormal.negate();
            surface.VN.push_back(transformedNormal);
        }
    }

    // 生成三角面片（与原代码一致）
    for (unsigned i = 0; i < steps; ++i) {
        unsigned next_i = (i + 1) % steps;
        for (size_t j = 0; j + 1 < profile.size(); ++j) {
            unsigned current = i * profile.size() + j;
            unsigned next = next_i * profile.size() + j;
            unsigned current_next = i * profile.size() + j + 1;
            unsigned next_next = next_i * profile.size() + j + 1;

            // surface.VF.push_back(Tup3u(current, next, current_next));
            // surface.VF.push_back(Tup3u(next, next_next, current_next));
            surface.VF.push_back(Tup3u(current, current_next, next));
            surface.VF.push_back(Tup3u(next, current_next, next_next));
        }
    }

    cerr << "\t>>> makeSurfRev called. Returning surface with "
         << surface.VV.size() << " vertices, "
         << surface.VN.size() << " normals, and "
         << surface.VF.size() << " faces." << endl;

    return surface;
}



#if 0
Surface makeGenCyl(const Curve &profile, const Curve &sweep )
{
    Surface surface;
	surface = quad();

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // TODO: Here you should build the surface.  See surf.h for details.

    cerr << "\t>>> makeGenCyl called (but not implemented).\n\t>>> Returning empty surface." <<endl;

    return surface;
}
#endif

Surface makeGenCyl(const Curve &profile, const Curve &sweep)
{
    Surface surface;

    // 检查轮廓曲线是否在 xy 平面上
    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // 初始化表面数据
    surface.VV.clear();
    surface.VN.clear();
    surface.VF.clear();

    // 遍历扫掠曲线的点
    for (size_t i = 0; i < sweep.size(); ++i)
    {
        // 获取当前扫掠曲线的 Frenet 框架（T, N, B）
        Vector3f sweepPoint = sweep[i].V;
        Vector3f sweepTangent = sweep[i].T;
        Vector3f sweepNormal = sweep[i].N;
        Vector3f sweepBinormal = sweep[i].B;

        // 遍历轮廓曲线的点
        for (size_t j = 0; j < profile.size(); ++j)
        {
            // 变换顶点位置
            Vector3f profilePoint = profile[j].V;
            Vector3f transformedPoint = sweepPoint +
                                        profilePoint.x() * sweepBinormal +
                                        profilePoint.y() * sweepNormal +
                                        profilePoint.z() * sweepTangent;
            surface.VV.push_back(transformedPoint);

            // 变换法向量（使用 Frenet 框架）
            Vector3f profileNormal = profile[j].N;
            Vector3f transformedNormal = profileNormal.x() * sweepBinormal +
                                        profileNormal.y() * sweepNormal +
                                        profileNormal.z() * sweepTangent;
            transformedNormal.normalize();  // 确保单位长度
            transformedNormal.negate();
            surface.VN.push_back(transformedNormal);
        }
    }

    // 生成三角面片（与原代码一致）
    for (size_t i = 0; i + 1 < sweep.size(); ++i)
    {
        for (size_t j = 0; j + 1 < profile.size(); ++j)
        {
            unsigned current = i * profile.size() + j;
            unsigned next = (i + 1) * profile.size() + j;
            unsigned current_next = i * profile.size() + j + 1;
            unsigned next_next = (i + 1) * profile.size() + j + 1;

            surface.VF.push_back(Tup3u(current, next, current_next));
            surface.VF.push_back(Tup3u(next, next_next, current_next));
        }
    }

    cerr << "\t>>> makeGenCyl called. Returning surface with "
         << surface.VV.size() << " vertices, "
         << surface.VN.size() << " normals, and "
         << surface.VF.size() << " faces." << endl;

    return surface;
}

void recordSurface(const Surface &surface, VertexRecorder* recorder) {
	const Vector3f WIRECOLOR(0.4f, 0.4f, 0.4f);
    for (int i=0; i<(int)surface.VF.size(); i++)
    {
		recorder->record(surface.VV[surface.VF[i][0]], surface.VN[surface.VF[i][0]], WIRECOLOR);
		recorder->record(surface.VV[surface.VF[i][1]], surface.VN[surface.VF[i][1]], WIRECOLOR);
		recorder->record(surface.VV[surface.VF[i][2]], surface.VN[surface.VF[i][2]], WIRECOLOR);
    }
}

void recordNormals(const Surface &surface, VertexRecorder* recorder, float len)
{
	const Vector3f NORMALCOLOR(0, 1, 1);
    for (int i=0; i<(int)surface.VV.size(); i++)
    {
		recorder->record_poscolor(surface.VV[i], NORMALCOLOR);
		recorder->record_poscolor(surface.VV[i] + surface.VN[i] * len, NORMALCOLOR);
    }
}

void outputObjFile(ostream &out, const Surface &surface)
{
    
    for (int i=0; i<(int)surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (int i=0; i<(int)surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;
    
    for (int i=0; i<(int)surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}
