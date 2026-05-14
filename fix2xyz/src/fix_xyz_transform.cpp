// #define DEBUG_MODE
#include "fix_xyz_transform.h"


using namespace fix_xyz_trans;
using namespace fix_utm_trans;


// クォータニオンから回転行列を計算する関数.
Eigen::Matrix3d RotMatFromQuat(const Eigen::Vector4d& quat){
    Eigen::Matrix3d Rot;
        Rot << 2*(quat(0)*quat(0) + quat(3)*quat(3))-1 , 2*(quat(0)*quat(1) - quat(2)*quat(3)), 2*(quat(0)*quat(2) + quat(1)*quat(3)) ,
        2*(quat(0)*quat(1) + quat(2)*quat(3)) , 2*(quat(1)*quat(1) + quat(3)*quat(3))-1 , 2*(quat(1)*quat(2) - quat(0)*quat(3)) ,
        2*(quat(0)*quat(2) - quat(1)*quat(3)) , 2*(quat(1)*quat(2) + quat(0)*quat(3)) , 2*(quat(2)*quat(2) + quat(3)*quat(3))-1 ;

    return Rot;
}

// クォータニオンからクォータニオン更新行列を計算する関数.
Eigen::Matrix4d QuatMatFromQuat(const Eigen::Vector4d& quat){
    Eigen::Matrix4d Mat_Q;
    Mat_Q << quat(3) , -quat(2) , quat(1) , quat(0) ,
        quat(2) , quat(3) , -quat(0) , quat(1) ,
        -quat(1) , quat(0) , quat(3) , quat(2) ,
        -quat(0) , -quat(1) , -quat(2) , quat(3) ;

    return Mat_Q;
}



LLAXYZTrans::LLAXYZTrans() : lla_utm_transformer(true) {
    set_origin_flg = false;
    set_origin_vector_flg = false;
    orig_R = Eigen::Matrix3d::Identity();
    orig_Q = Eigen::Matrix4d::Identity();
}
LLAXYZTrans::~LLAXYZTrans() {
}

void LLAXYZTrans::set_origin(LatLonAlt orig_lla_, Eigen::Vector4d orig_quat_){
    set_origin_flg = true;
    orig_lla = orig_lla_;
    orig_R = RotMatFromQuat(orig_quat_);
    orig_Q = QuatMatFromQuat(orig_quat_);

    UTM orig_utm = lla_utm_transformer.get_utm_from_latlonalt(orig_lla);
    orig_xyz << orig_utm.x, orig_utm.y, orig_utm.z;
    utm_zone = orig_utm.zone;
    DEBUG_PRINT(orig_xyz);

    set_origin_vector_flg = true;
    std::cout << std::fixed << "Origin vector in utm_" << utm_zone << " : \n" << orig_xyz << std::endl;
}

Eigen::Vector3d LLAXYZTrans::get_xyz_from_latlonalt(LatLonAlt latlonalt){
    std::string fix_debug_msg = "get_fix";
    DEBUG_PRINT(fix_debug_msg);

    UTM utm;
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    
    int utm_zone;

    if(!set_origin_flg || !set_origin_vector_flg){
        double inf = std::numeric_limits<double>::infinity();
        xyz << inf, inf, inf;
        return xyz;
    }
    
    utm = lla_utm_transformer.get_utm_from_latlonalt(latlonalt);
    xyz << utm.x, utm.y, utm.z;
    xyz = xyz - orig_xyz;
    xyz = orig_R * xyz;

    return xyz;
}


Pose LLAXYZTrans::get_xyz_from_latlonalt(LLAWithOrientation latlonalt){
    Pose ret_pose;
    ret_pose.position = get_xyz_from_latlonalt(latlonalt.lla);
    ret_pose.orientation = orig_Q * latlonalt.orientation;
    return ret_pose;
}

LatLonAlt LLAXYZTrans::get_latlonalt_from_xyz(Eigen::Vector3d xyz){
    std::string fix_debug_msg = "get_xyz";
    DEBUG_PRINT(fix_debug_msg);

    LatLonAlt latlonalt;
    UTM utm;

    double inf = std::numeric_limits<double>::infinity();
    if(!set_origin_flg){
        latlonalt.latitude = inf;
        latlonalt.longitude = inf;
        latlonalt.altitude = inf;
        return latlonalt;
    }

    xyz = orig_R.transpose() * xyz;
    xyz = xyz + orig_xyz;
    utm.x = xyz(0);
    utm.y = xyz(1);
    utm.z = xyz(2);
    utm.zone = utm_zone;

    latlonalt = lla_utm_transformer.get_latlonalt_from_utm(utm);

    fix_debug_msg = "return fix from xyz";
    DEBUG_PRINT(fix_debug_msg);

    return latlonalt;
}

LLAWithOrientation LLAXYZTrans::get_latlonalt_from_xyz(Pose xyz){
    LLAWithOrientation ret_lla;
    ret_lla.lla = get_latlonalt_from_xyz(xyz.position);
    ret_lla.orientation = orig_Q.transpose() * xyz.orientation;
    return ret_lla;
}