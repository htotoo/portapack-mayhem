/*
This is the protocol list handler. It holds an instance of all known protocols.
So include here the .hpp, and add a new element to the protos vector in the constructor. That's all you need to do here if you wanna add a new proto.
    @htotoo
*/

#include <vector>
#include <memory>
#include "portapack_shared_memory.hpp"

#include "fprotolistgeneral.hpp"
#include "subtpmsbase.hpp"

#include "t-schrader.hpp"
#include "t-ford.hpp"
#include "t-hyundai.hpp"
#include "t-abarth124.hpp"
#include "t-airpuxem.hpp"
#include "t-ave.hpp"
#include "t-bmw.hpp"
#include "t-elantra2012.hpp"
#include "t-renault_0435r.hpp"
#include "t-toyota.hpp"

#ifndef __FPROTO_PROTOLISTTPMS_H__
#define __FPROTO_PROTOLISTTPMS_H__

class SubTPMSProtos : public FProtoListGeneral {
   public:
    SubTPMSProtos(const SubTPMSProtos&) = delete;
    SubTPMSProtos& operator=(const SubTPMSProtos&) = delete;
    SubTPMSProtos() {
        // add protos
        protos[FPT_Schrader_EG53MA4] = new FProtoSubTPMSSchraderEG53MA4();
        protos[FPT_Schrader] = new FProtoSubTPMSSchrader();
        protos[FPT_Schrader_SMD3MA4] = new FProtoSubTPMSSchraderSMD3MA4();
        protos[FPT_Ford] = new FProtoSubTPMSFord();

        protos[FPT_HyundaiVDO] = new FProtoSubTPMS_VDO();
        protos[FPT_Abarth124] = new FProtoSubTPMSAbarth124();
        protos[FPT_Q85] = nullptr;  // implemented in the prev
        protos[FPT_Airpuxem] = new FProtoSubTPMSAirpuxem();
        protos[FPT_AVE] = new FProtoSubTPMSAVE();
        protos[FPT_BMW] = new FProtoSubTPMSBMW();
        protos[FPT_AUDI] = nullptr;     // in bmw
        protos[FPT_Citroen] = nullptr;  // implemented in vdo
        protos[FPT_Elantra2012] = new FProtoSubTPMSElantra2012();
        protos[FPT_Renault_0435R] = new FProtoSubTPMSRenault0435R();
        protos[FPT_Toyota] = new FProtoSubTPMSToyota();

        for (uint8_t i = 0; i < FPT_COUNT; ++i) {
            if (protos[i] != NULL) protos[i]->setCallback(callbackTarget);
        }
    }

    ~SubTPMSProtos() {  // not needed for current operation logic, but a bit more elegant :)
        for (uint8_t i = 0; i < FPT_COUNT; ++i) {
            if (protos[i] != NULL) {
                free(protos[i]);
                protos[i] = NULL;
            }
        }
    };

    static void callbackTarget(FProtoSubTPMSBase* instance) {
        SubTPMSDataMessage packet_message{instance->sensorType, instance->data_count_bit, instance->decode_data, instance->id, instance->battery, instance->temperature, instance->pressure};
        shared_memory.application_queue.push(packet_message);
    }

    void feed(bool level, uint32_t duration) {
        for (uint8_t i = 0; i < FPT_COUNT; ++i) {
            if (protos[i] != NULL) protos[i]->feed(level, duration);
        }
    }

   protected:
    FProtoSubTPMSBase* protos[FPT_COUNT] = {NULL};
};

#endif
