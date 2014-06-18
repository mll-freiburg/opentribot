
#ifndef _TribotsTools_DistanceCalibration_aux_h
#define _TribotsTools_DistanceCalibration_aux_h

namespace TribotsTools{
  enum DCMouseMode { SetCenterMode, SetDirectionMode, SetBlueMode, SetRedMode, SetAddMaskMode, SetSubMaskMode, SetBalanceMode };
  enum DCMode { DCNormal, DCMaskGeneration, DCMarkerCollection };
  struct DCAutoInfo {
    bool defaultCameraWhiteBalance;  ///< ist Kamera-Weissabgleich normalerweise an?
    bool defaultCameraShutter;
    bool defaultCameraGain;
    bool defaultSoftwareWhiteBalance;  ///< ist Software-Weissabgleich normalerweise an?
    bool defaultSoftwareExposure;

    bool useAuto;  ///< sind die Automatismen eingeschaltet?
  };
}

#endif
