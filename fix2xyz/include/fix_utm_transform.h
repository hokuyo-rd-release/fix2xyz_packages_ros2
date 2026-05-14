#ifndef FIX_UTM_TRANS
#define FIX_UTM_TRANS

#include <string>
#include <proj.h>
#include <limits>
#include <iostream>
#include <cmath>

#ifdef DEBUG_MODE
#define DEBUG_PRINT(A) std::cout << (#A) << " : " << (A) << std::endl;
#else
#define DEBUG_PRINT(A) {;}
#endif


namespace fix_utm_trans{

    // 緯度経度を表す構造体.
    struct LatLonAlt {
        double latitude;
        double longitude;
        double altitude;
        LatLonAlt(){
            latitude = 0;
            longitude = 0;
            altitude = 0;
        }
    };

    // UTMゾーンとUTM座標を表す構造体.
    struct UTM {
        int zone;
        double x;
        double y;
        double z;
    };


    class LLAUTMTrans {
    private:

        bool use_first_zone;
        PJ_CONTEXT* ctx;
        PJ* pj;

        // 経度からUTMゾーンを取得する関数.
        int judge_utm_zone(double longitude);

        // UTMゾーンをEPSGコードに変換する関数.
        std::string utm_zone_to_epsg(int utm_zone);

        // 新しい座標系を設定する関数.
        void set_coord(const std::string& src_crs, const std::string& dst_crs);
        
    protected:
    
        int first_zone;
        int last_zone;
    

    public:

        // 緯度経度からUTMゾーンとUTM座標を求める関数.
        UTM get_utm_from_latlonalt(LatLonAlt latlonalt);

        // UTMゾーンとUTM座標から緯度経度を求める関数.
        LatLonAlt get_latlonalt_from_utm(UTM utm);

        LLAUTMTrans(bool zone_is_static);
        ~LLAUTMTrans();

    };
}


#endif