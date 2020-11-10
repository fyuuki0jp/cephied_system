
#include <Arduino.h>
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif
#include "kpu.h"
#include "dmac.h"
#ifdef __cplusplus
}
#endif


class CKPUDevice{
    public:
        CKPUDevice(const dmac_channel_number_t useDMA_CH,uint8_t* binary);
        ~CKPUDevice();
        int runForward(const uint8_t* src,bool fSync = true);
        bool isDone();
        int getOutput(int index,uint8_t** data,size_t* size);
        bool m_fDone;
        uint8_t* m_ModelData;
        kpu_model_context_t m_task;
        dmac_channel_number_t m_useDMA_CH;
};