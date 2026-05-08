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

#ifndef __FPROTO_PROTOLISTTPMS_H__
#define __FPROTO_PROTOLISTTPMS_H__

class SubTPMSProtos : public FProtoListGeneral {
   public:
    SubTPMSProtos(const SubTPMSProtos&) = delete;
    SubTPMSProtos& operator=(const SubTPMSProtos&) = delete;
    SubTPMSProtos() {
        // add protos
        protos[FPT_Schrader] = new FProtoSubTPMSSchrader();

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
