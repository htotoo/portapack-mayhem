
#ifndef __FPROTO_SUBTPMSTYPES_H__
#define __FPROTO_SUBTPMSTYPES_H__

/*
Define known protocols.
These values must be present on the protocol's constructor, like FProtoWeatherAcurite592TXR()  {   sensorType = FPS_ANSONIC;     }
Also it must have a switch-case element in the getSubGhzDSensorTypeName() function, to display it's name.
*/

enum FPROTO_SUBTPMS_SENSOR : uint8_t {
    FPT_Invalid = 0,
    FPT_Schrader_EG53MA4 = 1,
    FPT_Schrader = 2,
    FPT_Schrader_SMD3MA4 = 3,
    FPT_Ford = 4,
    FPT_HyundaiVDO = 5,
    FPT_Abarth124 = 6,
    FPT_Q85 = 7,
    FPT_Airpuxem = 8,
    FPT_AVE = 9,
    FPT_BMW = 10,
    FPT_AUDI = 11,
    FPT_Citroen = 12,
    FPT_Elantra2012 = 13,
    FPT_Renault_0435R = 14,
    FPT_Toyota = 15,
    FPT_COUNT
};

#endif
