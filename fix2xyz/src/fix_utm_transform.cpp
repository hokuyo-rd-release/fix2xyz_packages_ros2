#include "fix_utm_transform.h"

using namespace fix_utm_trans;

LLAUTMTrans::LLAUTMTrans(bool zone_is_static) {
    use_first_zone = zone_is_static;
    
    ctx =  proj_context_create();
    pj = nullptr;

    first_zone = -1;
    last_zone = -1;
}

LLAUTMTrans::~LLAUTMTrans() {
    if (pj) proj_destroy(pj);
    if (ctx) proj_context_destroy(ctx);
}

void LLAUTMTrans::set_coord(const std::string& src_crs, const std::string& dst_crs) {
    if (pj) {
        proj_destroy(pj);   // 古いPJを破棄
        pj = nullptr;
    }
    pj = proj_create_crs_to_crs(ctx, src_crs.c_str(), dst_crs.c_str(), nullptr);
    if (!pj) {
        throw std::runtime_error("Failed to create PJ");
    }
}

int LLAUTMTrans::judge_utm_zone(double longitude){
    //経度を6で割って切り上げ.
    int zone = (int)(longitude + 180.0 + 5 )/6;
    return zone;
}

std::string LLAUTMTrans::utm_zone_to_epsg(int utm_zone){
    int epsg_num = utm_zone + 32600;
    return "EPSG:" + std::to_string(epsg_num);
}

UTM LLAUTMTrans::get_utm_from_latlonalt(LatLonAlt latlonalt){
    std::string latlonalt_debug_msg = "get_latlonalt";
    DEBUG_PRINT(latlonalt_debug_msg);

    UTM utm;
    
    int utm_zone = judge_utm_zone(latlonalt.longitude);

    // utm_zoneが不正値.
    if(!(utm_zone > 0 && utm_zone <=60)){
        double inf = std::numeric_limits<double>::infinity();
        utm.x = inf;
        utm.y = inf;
        utm.z = inf;
        utm.zone = -1;
        return utm;
    }
    
    std::string epsg_code;

    // first_zoneの設定.
    if(!(first_zone > 0 && first_zone <=60)){
        first_zone = utm_zone;
    }
    if(use_first_zone){
        utm_zone = first_zone;
    }
    // pjオブジェクトの 作成 || 更新.
    if(!pj || (!use_first_zone && ( last_zone != utm_zone))){
        epsg_code = utm_zone_to_epsg(utm_zone);
        set_coord("EPSG:4326", epsg_code);
    }

    DEBUG_PRINT(utm_zone);
    DEBUG_PRINT(epsg_code);
    DEBUG_PRINT(latlonalt.latitude);
    DEBUG_PRINT(latlonalt.longitude);

    if (!pj) {
        double inf = std::numeric_limits<double>::infinity();
        utm.x = inf;
        utm.y = inf;
        utm.z = inf;
        utm.zone = -1;
    }
    else{
        PJ_COORD a = proj_coord(latlonalt.latitude, latlonalt.longitude, 0, 0);
        PJ_COORD b = proj_trans(pj, PJ_FWD, a);

        utm.x = b.xy.x;
        utm.y = b.xy.y;
        utm.z = latlonalt.altitude;
        utm.zone = utm_zone;
    }
    
    last_zone = utm_zone;
    latlonalt_debug_msg = "return utm from latlonalt";
    DEBUG_PRINT(latlonalt_debug_msg);

    return utm;
}

LatLonAlt LLAUTMTrans::get_latlonalt_from_utm(UTM utm){
    std::string latlonalt_debug_msg = "get_utm";
    DEBUG_PRINT(latlonalt_debug_msg);

    LatLonAlt latlonalt;

    std::string epsg_code;

    // 座標値が不正値.
    bool is_invalid_position = !std::isfinite(utm.x)|| !std::isfinite(utm.y);

    // utm_zoneが不正値.
    bool is_invalid_zone = !(utm.zone > 0 && utm.zone <=60);
    
    if(is_invalid_position || is_invalid_zone){
        double inf = std::numeric_limits<double>::infinity();
        latlonalt.latitude = inf;
        latlonalt.longitude = inf;
        latlonalt.altitude = inf;
        return latlonalt;
    }

    // PROJオブジェクトの座標系設定.
    if(!(first_zone > 0 && first_zone <=60)){
        first_zone = utm.zone;
    }
    if(!pj || (!use_first_zone && ( last_zone != utm.zone))){
        epsg_code = utm_zone_to_epsg(utm.zone);
        set_coord("EPSG:4326", epsg_code);
    }

    DEBUG_PRINT(utm.zone);
    DEBUG_PRINT(epsg_code);
    DEBUG_PRINT(utm.x);
    DEBUG_PRINT(utm.y);

    if (!pj) {
        double inf = std::numeric_limits<double>::infinity();
        latlonalt.latitude = inf;
        latlonalt.longitude = inf;
        latlonalt.altitude = inf;
    }
    else{
        PJ_COORD a = proj_coord(utm.x, utm.y, 0, 0);
        PJ_COORD b = proj_trans(pj, PJ_INV, a);

        latlonalt.latitude = b.xy.x;
        latlonalt.longitude = b.xy.y;
        latlonalt.altitude = utm.z;
    }

    last_zone = utm.zone;
    latlonalt_debug_msg = "return latlonalt from utm";
    DEBUG_PRINT(latlonalt_debug_msg);

    return latlonalt;
}
