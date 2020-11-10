#include "KPUDevice.h"

CKPUDevice::CKPUDevice(const dmac_channel_number_t useDMA_CH,uint8_t* binary)
    :m_fDone(false)
{
    m_useDMA_CH = useDMA_CH;
    m_ModelData = binary;
    if (kpu_load_kmodel(&m_task, m_ModelData) != 0)
    {
        Serial.printf("\nmodel init error\n");
    }
    m_task.userdata = (void*)this;

}

static void ai_done(void* ctx)
{
    CKPUDevice* pParent = (CKPUDevice*)ctx;
    pParent->m_fDone = true;
}

bool CKPUDevice::isDone()
{
    return m_fDone;
}

int CKPUDevice::runForward(const uint8_t* src,bool fSync)
{
    m_fDone = false;
    int ret = kpu_run_kmodel(&m_task, src, m_useDMA_CH, ai_done, this);
    return ret;
}

int CKPUDevice::getOutput(int index,uint8_t** data,size_t* size)
{
    if(m_fDone){
        return kpu_get_output(&m_task, 0, data, size);
    }
    else
    {
        return -1;
    }
    
}