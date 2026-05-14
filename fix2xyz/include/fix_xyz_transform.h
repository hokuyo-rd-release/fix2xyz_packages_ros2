#ifndef LATLON_UTM_TRANS
#define LATLON_UTM_TRANS


#ifdef DEBUG_MODE
#define DEBUG_PRINT(A) std::cout << (#A) << " : " << (A) << std::endl;
#else
#define DEBUG_PRINT(A) {;}
#endif

#include <string>
#include <proj.h>
#include <limits>
#include <iostream>
#include <Eigen/Dense>
#include "fix_utm_transform.h"

using namespace fix_utm_trans;
namespace fix_xyz_trans{

    // 緯度経度に角度を足した構造体.
    struct LLAWithOrientation {
        LatLonAlt lla;
        Eigen::Vector4d orientation;
        LLAWithOrientation(){
            orientation << 0,0,0,1;
        }
    };

    struct Pose {
        Eigen::Vector3d position;
        Eigen::Vector4d orientation;
        Pose(){
            position = Eigen::Vector3d::Zero();
            orientation << 0,0,0,1;
        }
    };

    class LLAXYZTrans {
    private:
        LLAUTMTrans lla_utm_transformer;

        bool set_origin_flg;
        bool set_origin_vector_flg;
        int utm_zone;

        LatLonAlt orig_lla;
        Eigen::Matrix3d orig_R;    // 緯度経度の基準姿勢.
        Eigen::Matrix4d orig_Q;
        Eigen::Vector3d orig_xyz;
        
    protected:

    public:

        // 緯度経度からUTMゾーンとUTM座標を求める関数.
        Eigen::Vector3d get_xyz_from_latlonalt(LatLonAlt latlonalt);
        Pose get_xyz_from_latlonalt(LLAWithOrientation latlonalt);

        LatLonAlt get_latlonalt_from_xyz(Eigen::Vector3d xyz);
        LLAWithOrientation get_latlonalt_from_xyz(Pose xyz);

        void set_origin( LatLonAlt orig_lla_, Eigen::Vector4d orig_quat_ );

        LLAXYZTrans();
        ~LLAXYZTrans();

    };
}


#endif
