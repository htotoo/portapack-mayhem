
#ifndef __FPROTO_SUBTPMSTYPES_H__
#define __FPROTO_SUBTPMSTYPES_H__

/*
Define known protocols.
These values must be present on the protocol's constructor, like FProtoWeatherAcurite592TXR()  {   sensorType = FPS_ANSONIC;     }
Also it must have a switch-case element in the getSubGhzDSensorTypeName() function, to display it's name.
*/

enum FPROTO_SUBTPMS_SENSOR : uint8_t {
    FPT_Invalid = 0,
    FPT_Schrader = 1,
    FPT_Ford = 2,
    FPT_HyundaiVDO = 3,
    FPT_Abarth124 = 4,
    FPT_Q85 = 5,
    FPT_Airpuxem = 6,
    FPT_AVE = 7,
    FPT_BMW = 8,
    FPT_AUDI = 9,
    FPT_COUNT
};

#endif
