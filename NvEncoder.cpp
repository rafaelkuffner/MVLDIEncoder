////////////////////////////////////////////////////////////////////////////
//
// Copyright 1993-2014 NVIDIA Corporation.  All rights reserved.
//
// Please refer to the NVIDIA end user license agreement (EULA) associated
// with this source code for terms and conditions that govern your use of
// this software. Any use, reproduction, disclosure, or distribution of
// this software and related documentation outside the terms of the EULA
// is strictly prohibited.
//
////////////////////////////////////////////////////////////////////////////

#include "nvCPUOPSys.h"
#include "nvEncodeAPI.h"
#include "nvUtils.h"
#include "NvEncoder.h"
#include "nvFileIO.h"
#include <new>
#include <iostream>

#define BITSTREAM_BUFFER_SIZE 2 * 1024 * 1024

void* CNvEncoder::m_pDevice = NULL;

CNvEncoder::CNvEncoder()
{
    m_pNvHWEncoder = new CNvHWEncoder;
#if defined (NV_WINDOWS)
    m_pD3D = NULL;
#endif
    m_cuContext = NULL;

    m_uEncodeBufferCount = 0;
    memset(&m_stEncoderInput, 0, sizeof(m_stEncoderInput));
    memset(&m_stEOSOutputBfr, 0, sizeof(m_stEOSOutputBfr));
    memset(&m_stMVBuffer, 0, sizeof(m_stMVBuffer));
    memset(&m_stEncodeBuffer, 0, sizeof(m_stEncodeBuffer));
}

CNvEncoder::~CNvEncoder()
{
    if (m_pNvHWEncoder)
    {
        delete m_pNvHWEncoder;
        m_pNvHWEncoder = NULL;
    }
}

NVENCSTATUS CNvEncoder::InitCuda(uint32_t deviceID)
{
    CUresult cuResult;
    CUdevice device;
    CUcontext cuContextCurr;
    int  deviceCount = 0;
    int  SMminor = 0, SMmajor = 0;

#if defined(WIN32) || defined(_WIN32) || defined(WIN64) || defined(_WIN64)
    typedef HMODULE CUDADRIVER;
#else
    typedef void *CUDADRIVER;
#endif
    CUDADRIVER hHandleDriver = 0;
    cuResult = cuInit(0, __CUDA_API_VERSION, hHandleDriver);
    if (cuResult != CUDA_SUCCESS)
    {
        PRINTERR("cuInit error:0x%x\n", cuResult);
        assert(0);
        return NV_ENC_ERR_NO_ENCODE_DEVICE;
    }

    cuResult = cuDeviceGetCount(&deviceCount);
    if (cuResult != CUDA_SUCCESS)
    {
        PRINTERR("cuDeviceGetCount error:0x%x\n", cuResult);
        assert(0);
        return NV_ENC_ERR_NO_ENCODE_DEVICE;
    }

    // If dev is negative value, we clamp to 0
    if ((int)deviceID < 0)
        deviceID = 0;

    if (deviceID >(unsigned int)deviceCount - 1)
    {
        PRINTERR("Invalid Device Id = %d\n", deviceID);
        return NV_ENC_ERR_INVALID_ENCODERDEVICE;
    }

    cuResult = cuDeviceGet(&device, deviceID);
    if (cuResult != CUDA_SUCCESS)
    {
        PRINTERR("cuDeviceGet error:0x%x\n", cuResult);
        return NV_ENC_ERR_NO_ENCODE_DEVICE;
    }

	char *name = (char*)malloc(256 * sizeof(char));
	cuResult = cuDeviceGetName(name, 256, device);
	std::cout << "Device name: " << name << std::endl;
    cuResult = cuDeviceComputeCapability(&SMmajor, &SMminor, deviceID);
    if (cuResult != CUDA_SUCCESS)
    {
        PRINTERR("cuDeviceComputeCapability error:0x%x\n", cuResult);
        return NV_ENC_ERR_NO_ENCODE_DEVICE;
    }

    if (((SMmajor << 4) + SMminor) < 0x30)
    {
        PRINTERR("GPU %d does not have NVENC capabilities exiting\n", deviceID);
        return NV_ENC_ERR_NO_ENCODE_DEVICE;
    }

    cuResult = cuCtxCreate((CUcontext*)(&m_pDevice), 0, device);
    if (cuResult != CUDA_SUCCESS)
    {
        PRINTERR("cuCtxCreate error:0x%x\n", cuResult);
        assert(0);
        return NV_ENC_ERR_NO_ENCODE_DEVICE;
    }

    cuResult = cuCtxPopCurrent(&cuContextCurr);
    if (cuResult != CUDA_SUCCESS)
    {
        PRINTERR("cuCtxPopCurrent error:0x%x\n", cuResult);
        assert(0);
        return NV_ENC_ERR_NO_ENCODE_DEVICE;
    }
    return NV_ENC_SUCCESS;
}

NVENCSTATUS CNvEncoder::AllocateIOBuffers(uint32_t uInputWidth, uint32_t uInputHeight, NV_ENC_BUFFER_FORMAT inputFormat)
{
    NVENCSTATUS nvStatus = NV_ENC_SUCCESS;

    m_EncodeBufferQueue.Initialize(m_stEncodeBuffer, m_uEncodeBufferCount);
    for (uint32_t i = 0; i < m_uEncodeBufferCount; i++)
    {
        nvStatus = m_pNvHWEncoder->NvEncCreateInputBuffer(uInputWidth, uInputHeight, &m_stEncodeBuffer[i].stInputBfr.hInputSurface, inputFormat);
        if (nvStatus != NV_ENC_SUCCESS)
            return nvStatus;

        m_stEncodeBuffer[i].stInputBfr.bufferFmt = inputFormat;
        m_stEncodeBuffer[i].stInputBfr.dwWidth = uInputWidth;
        m_stEncodeBuffer[i].stInputBfr.dwHeight = uInputHeight;
        nvStatus = m_pNvHWEncoder->NvEncCreateBitstreamBuffer(BITSTREAM_BUFFER_SIZE, &m_stEncodeBuffer[i].stOutputBfr.hBitstreamBuffer);
        if (nvStatus != NV_ENC_SUCCESS)
            return nvStatus;
         m_stEncodeBuffer[i].stOutputBfr.dwBitstreamBufferSize = BITSTREAM_BUFFER_SIZE;
        if (m_stEncoderInput.enableAsyncMode)
        {
            nvStatus = m_pNvHWEncoder->NvEncRegisterAsyncEvent(&m_stEncodeBuffer[i].stOutputBfr.hOutputEvent);
            if (nvStatus != NV_ENC_SUCCESS)
                return nvStatus;
            m_stEncodeBuffer[i].stOutputBfr.bWaitOnEvent = true;
        }
        else
            m_stEncodeBuffer[i].stOutputBfr.hOutputEvent = NULL;
    }

    m_stEOSOutputBfr.bEOSFlag = TRUE;

    if (m_stEncoderInput.enableAsyncMode)
    {
        nvStatus = m_pNvHWEncoder->NvEncRegisterAsyncEvent(&m_stEOSOutputBfr.hOutputEvent);
        if (nvStatus != NV_ENC_SUCCESS)
            return nvStatus;
    }
    else
        m_stEOSOutputBfr.hOutputEvent = NULL;

    return NV_ENC_SUCCESS;
}

NVENCSTATUS CNvEncoder::ReleaseIOBuffers()
{
    for (uint32_t i = 0; i < m_uEncodeBufferCount; i++)
    {
        m_pNvHWEncoder->NvEncDestroyInputBuffer(m_stEncodeBuffer[i].stInputBfr.hInputSurface);
        m_stEncodeBuffer[i].stInputBfr.hInputSurface = NULL;
        m_pNvHWEncoder->NvEncDestroyBitstreamBuffer(m_stEncodeBuffer[i].stOutputBfr.hBitstreamBuffer);
        m_stEncodeBuffer[i].stOutputBfr.hBitstreamBuffer = NULL;
        if (m_stEncoderInput.enableAsyncMode)
        {
            m_pNvHWEncoder->NvEncUnregisterAsyncEvent(m_stEncodeBuffer[i].stOutputBfr.hOutputEvent);
            nvCloseFile(m_stEncodeBuffer[i].stOutputBfr.hOutputEvent);
            m_stEncodeBuffer[i].stOutputBfr.hOutputEvent = NULL;
        }
    }

    if (m_stEOSOutputBfr.hOutputEvent)
    {
        if (m_stEncoderInput.enableAsyncMode)
        {
            m_pNvHWEncoder->NvEncUnregisterAsyncEvent(m_stEOSOutputBfr.hOutputEvent);
            nvCloseFile(m_stEOSOutputBfr.hOutputEvent);
            m_stEOSOutputBfr.hOutputEvent = NULL;
        }
    }

    return NV_ENC_SUCCESS;
}

NVENCSTATUS CNvEncoder::FlushEncoder()
{
    NVENCSTATUS nvStatus = m_pNvHWEncoder->NvEncFlushEncoderQueue(m_stEOSOutputBfr.hOutputEvent);
    if (nvStatus != NV_ENC_SUCCESS)
    {
        assert(0);
        return nvStatus;
    }

    EncodeBuffer *pEncodeBufer = m_EncodeBufferQueue.GetPending();
    while (pEncodeBufer)
    {
        m_pNvHWEncoder->ProcessOutput(pEncodeBufer);
        pEncodeBufer = m_EncodeBufferQueue.GetPending();
    }

#if defined(NV_WINDOWS)
    if (m_stEncoderInput.enableAsyncMode)
    {
    
        if (WaitForSingleObject(m_stEOSOutputBfr.hOutputEvent, 500) != WAIT_OBJECT_0)
        {
            assert(0);
            nvStatus = NV_ENC_ERR_GENERIC;
        }
    }
#endif  

    return nvStatus;
}

NVENCSTATUS CNvEncoder::Deinitialize()
{
    NVENCSTATUS nvStatus = NV_ENC_SUCCESS;

    ReleaseIOBuffers();
   
    nvStatus = m_pNvHWEncoder->NvEncDestroyEncoder();


#if defined (NV_WINDOWS)
    if (m_pD3D)
    {
        m_pD3D->Release();
        m_pD3D = NULL;
    }
#endif

    return nvStatus;
}

void CNvEncoder::DeInitCuda() {

	
	if (m_pDevice)
	{
		CUresult cuResult = CUDA_SUCCESS;
		cuResult = cuCtxDestroy((CUcontext)m_pDevice);
		if (cuResult != CUDA_SUCCESS)
			PRINTERR("cuCtxDestroy error:0x%x\n", cuResult);
		m_pDevice = NULL;
	}
}

NVENCSTATUS loadframe(uint8_t *yuvInput[3], HANDLE hInputYUVFile, uint32_t frmIdx, uint32_t width, uint32_t height, uint32_t &numBytesRead, NV_ENC_BUFFER_FORMAT inputFormat)
{
    uint64_t fileOffset;
    uint32_t result;
    //Set size depending on whether it is YUV 444 or YUV 420
    uint32_t dwInFrameSize = 0;
    int anFrameSize[3] = {};
    switch (inputFormat) {
    default:
    case NV_ENC_BUFFER_FORMAT_NV12: 
        dwInFrameSize = width * height * 3 / 2; 
        anFrameSize[0] = width * height;
        anFrameSize[1] = anFrameSize[2] = width * height / 4;
        break;
    case NV_ENC_BUFFER_FORMAT_YUV444:
        dwInFrameSize = width * height * 3;
        anFrameSize[0] = anFrameSize[1] = anFrameSize[2] = width * height;
        break;
    case NV_ENC_BUFFER_FORMAT_YUV420_10BIT:
        dwInFrameSize = width * height * 3;
        anFrameSize[0] = width * height * 2;
        anFrameSize[1] = anFrameSize[2] = width * height / 2;
        break;
    case NV_ENC_BUFFER_FORMAT_YUV444_10BIT:
        dwInFrameSize = width * height * 6;
        anFrameSize[0] = anFrameSize[1] = anFrameSize[2] = width * height * 2;
        break;
    }
    fileOffset = (uint64_t)dwInFrameSize * frmIdx;
    result = nvSetFilePointer64(hInputYUVFile, fileOffset, NULL, FILE_BEGIN);
    if (result == INVALID_SET_FILE_POINTER)
    {
        return NV_ENC_ERR_INVALID_PARAM;
    }
    nvReadFile(hInputYUVFile, yuvInput[0], anFrameSize[0], &numBytesRead, NULL);
    nvReadFile(hInputYUVFile, yuvInput[1], anFrameSize[1], &numBytesRead, NULL);
    nvReadFile(hInputYUVFile, yuvInput[2], anFrameSize[2], &numBytesRead, NULL);
    return NV_ENC_SUCCESS;
}

void CNvEncoder::printStatus() {
	printf("Encoding input           : \"%s\"\n", m_encodeConfig.inputFileName);
	printf("         output          : \"%s\"\n", m_encodeConfig.outputFileName);
	printf("         codec           : \"%s\"\n", m_encodeConfig.codec == NV_ENC_HEVC ? "HEVC" : "H264");
	printf("         size            : %dx%d\n", m_encodeConfig.width, m_encodeConfig.height);
	printf("         bitrate         : %d bits/sec\n", m_encodeConfig.bitrate);
	printf("         vbvMaxBitrate   : %d bits/sec\n", m_encodeConfig.vbvMaxBitrate);
	printf("         vbvSize         : %d bits\n", m_encodeConfig.vbvSize);
	printf("         fps             : %d frames/sec\n", m_encodeConfig.fps);
	printf("         rcMode          : %s\n", m_encodeConfig.rcMode == NV_ENC_PARAMS_RC_CONSTQP ? "CONSTQP" :
		m_encodeConfig.rcMode == NV_ENC_PARAMS_RC_VBR ? "VBR" :
		m_encodeConfig.rcMode == NV_ENC_PARAMS_RC_CBR ? "CBR" :
		m_encodeConfig.rcMode == NV_ENC_PARAMS_RC_VBR_MINQP ? "VBR MINQP" :
		m_encodeConfig.rcMode == NV_ENC_PARAMS_RC_2_PASS_QUALITY ? "TWO_PASS_QUALITY" :
		m_encodeConfig.rcMode == NV_ENC_PARAMS_RC_2_PASS_FRAMESIZE_CAP ? "TWO_PASS_FRAMESIZE_CAP" :
		m_encodeConfig.rcMode == NV_ENC_PARAMS_RC_2_PASS_VBR ? "TWO_PASS_VBR" : "UNKNOWN");
	if (m_encodeConfig.gopLength == NVENC_INFINITE_GOPLENGTH)
		printf("         goplength       : INFINITE GOP \n");
	else
		printf("         goplength       : %d \n", m_encodeConfig.gopLength);
	printf("         B frames        : %d \n", m_encodeConfig.numB);
	printf("         QP              : %d \n", m_encodeConfig.qp);
	printf("       Input Format      : %s\n",
		m_encodeConfig.inputFormat == NV_ENC_BUFFER_FORMAT_NV12 ? "YUV 420" :
		(m_encodeConfig.inputFormat == NV_ENC_BUFFER_FORMAT_YUV444 ? "YUV 444" :
		(m_encodeConfig.inputFormat == NV_ENC_BUFFER_FORMAT_ARGB ? "ARGB" :"YUV 444 10-bit")));
	printf("         preset          : %s\n", (m_encodeConfig.presetGUID == NV_ENC_PRESET_LOW_LATENCY_HQ_GUID) ? "LOW_LATENCY_HQ" :
		(m_encodeConfig.presetGUID == NV_ENC_PRESET_LOW_LATENCY_HP_GUID) ? "LOW_LATENCY_HP" :
		(m_encodeConfig.presetGUID == NV_ENC_PRESET_HQ_GUID) ? "HQ_PRESET" :
		(m_encodeConfig.presetGUID == NV_ENC_PRESET_HP_GUID) ? "HP_PRESET" :
		(m_encodeConfig.presetGUID == NV_ENC_PRESET_LOSSLESS_HP_GUID) ? "LOSSLESS_HP" :
		(m_encodeConfig.presetGUID == NV_ENC_PRESET_LOW_LATENCY_DEFAULT_GUID) ? "LOW_LATENCY_DEFAULT" : "DEFAULT");
	printf("  Picture Structure      : %s\n", (m_encodeConfig.pictureStruct == NV_ENC_PIC_STRUCT_FRAME) ? "Frame Mode" :
		(m_encodeConfig.pictureStruct == NV_ENC_PIC_STRUCT_FIELD_TOP_BOTTOM) ? "Top Field first" :
		(m_encodeConfig.pictureStruct == NV_ENC_PIC_STRUCT_FIELD_BOTTOM_TOP) ? "Bottom Field first" : "INVALID");
	printf("         devicetype      : CUDA\n");

	printf("\n");
}

NVENCSTATUS CNvEncoder::EncodeFrame(EncodeFrameConfig *pEncodeFrame, bool bFlush, uint32_t width, uint32_t height)
{
    NVENCSTATUS nvStatus = NV_ENC_SUCCESS;
    uint32_t lockedPitch = 0;
    EncodeBuffer *pEncodeBuffer = NULL;

    if (bFlush)
    {
        FlushEncoder();
        return NV_ENC_SUCCESS;
    }

    if (!pEncodeFrame)
    {
        return NV_ENC_ERR_INVALID_PARAM;
    }

    pEncodeBuffer = m_EncodeBufferQueue.GetAvailable();
    if(!pEncodeBuffer)
    {
        m_pNvHWEncoder->ProcessOutput(m_EncodeBufferQueue.GetPending());
        pEncodeBuffer = m_EncodeBufferQueue.GetAvailable();
    }

	//Pointer to data
    unsigned char *pInputSurface;
    
    nvStatus = m_pNvHWEncoder->NvEncLockInputBuffer(pEncodeBuffer->stInputBfr.hInputSurface, (void**)&pInputSurface, &lockedPitch);
    if (nvStatus != NV_ENC_SUCCESS)
        return nvStatus;

    //if (pEncodeBuffer->stInputBfr.bufferFmt == NV_ENC_BUFFER_FORMAT_ARGB)
    //{
		memcpy(pInputSurface, pEncodeFrame->rgb, pEncodeFrame->height*pEncodeFrame->stride);
      //unsigned char *pInputSurfaceCb = pInputSurface + (pEncodeBuffer->stInputBfr.dwHeight * lockedPitch);
      //unsigned char *pInputSurfaceCr = pInputSurfaceCb + (pEncodeBuffer->stInputBfr.dwHeight * lockedPitch);
      //convertYUV10pitchtoYUV444((uint16_t *)pEncodeFrame->yuv[0], (uint16_t *)pEncodeFrame->yuv[1], (uint16_t *)pEncodeFrame->yuv[2], (uint16_t *)pInputSurface, (uint16_t *)pInputSurfaceCb, (uint16_t *)pInputSurfaceCr, width, height, width, lockedPitch);
    //}
    nvStatus = m_pNvHWEncoder->NvEncUnlockInputBuffer(pEncodeBuffer->stInputBfr.hInputSurface);
    if (nvStatus != NV_ENC_SUCCESS)
        return nvStatus;

    nvStatus = m_pNvHWEncoder->NvEncEncodeFrame(pEncodeBuffer, NULL, width, height, (NV_ENC_PIC_STRUCT)m_uPicStruct);
    return nvStatus;
}

bool CNvEncoder::InitEncoder(int width, int height, char *outputPath)
{

	NVENCSTATUS nvStatus = NV_ENC_SUCCESS;
    unsigned int preloadedFrameCount = FRAME_QUEUE;

    memset(&m_encodeConfig, 0, sizeof(m_encodeConfig));

    m_encodeConfig.endFrameIdx = INT_MAX;
    m_encodeConfig.bitrate = 5000000;
    m_encodeConfig.rcMode = NV_ENC_PARAMS_RC_CONSTQP; //LOSSLESS
    m_encodeConfig.gopLength = NVENC_INFINITE_GOPLENGTH;
    m_encodeConfig.codec = NV_ENC_H264;
    m_encodeConfig.fps = 30;
    m_encodeConfig.qp = 28; //LOSSLESS
    m_encodeConfig.i_quant_factor = DEFAULT_I_QFACTOR;
    m_encodeConfig.b_quant_factor = DEFAULT_B_QFACTOR;
    m_encodeConfig.i_quant_offset = DEFAULT_I_QOFFSET;
    m_encodeConfig.b_quant_offset = DEFAULT_B_QOFFSET; 
	m_encodeConfig.presetGUID = NV_ENC_PRESET_DEFAULT_GUID;
    m_encodeConfig.pictureStruct = NV_ENC_PIC_STRUCT_FRAME;
    m_encodeConfig.inputFormat = NV_ENC_BUFFER_FORMAT_ARGB;
	m_encodeConfig.outputFileName = outputPath;
	m_encodeConfig.width = width;
	m_encodeConfig.height = height;
	//m_encodeConfig.encoderPreset = "";

    m_encodeConfig.fOutput = fopen(m_encodeConfig.outputFileName, "wb");
    if (m_encodeConfig.fOutput == NULL)
    {
        PRINTERR("Failed to create \"%s\"\n", m_encodeConfig.outputFileName);
        return false;
    }


   nvStatus = m_pNvHWEncoder->Initialize(m_pDevice, NV_ENC_DEVICE_TYPE_CUDA);
    if (nvStatus != NV_ENC_SUCCESS)
        return false;

    m_encodeConfig.presetGUID = m_pNvHWEncoder->GetPresetGUID(m_encodeConfig.encoderPreset, m_encodeConfig.codec);

	printStatus();

    nvStatus = m_pNvHWEncoder->CreateEncoder(&m_encodeConfig);
    if (nvStatus != NV_ENC_SUCCESS)
        return false;
    m_encodeConfig.maxWidth = m_encodeConfig.maxWidth ? m_encodeConfig.maxWidth : m_encodeConfig.width;
    m_encodeConfig.maxHeight = m_encodeConfig.maxHeight ? m_encodeConfig.maxHeight : m_encodeConfig.height;

    m_stEncoderInput.enableAsyncMode = m_encodeConfig.enableAsyncMode;

	//NAO SEI AINDA ---------------------------------
    if (m_encodeConfig.numB > 0)
    {
        m_uEncodeBufferCount = m_encodeConfig.numB + 4; // min buffers is numb + 1 + 3 pipelining
    }
    else
    {
        int numMBs = ((m_encodeConfig.maxHeight + 15) >> 4) * ((m_encodeConfig.maxWidth + 15) >> 4);
        int NumIOBuffers;
        if (numMBs >= 32768) //4kx2k
            NumIOBuffers = MAX_ENCODE_QUEUE / 8;
        else if (numMBs >= 16384) // 2kx2k
            NumIOBuffers = MAX_ENCODE_QUEUE / 4;
        else if (numMBs >= 8160) // 1920x1080
            NumIOBuffers = MAX_ENCODE_QUEUE / 2;
        else
            NumIOBuffers = MAX_ENCODE_QUEUE;
        m_uEncodeBufferCount = NumIOBuffers;
    }
	//-----------------------------------------------

    m_uPicStruct = m_encodeConfig.pictureStruct;
    
    nvStatus = AllocateIOBuffers(m_encodeConfig.width, m_encodeConfig.height, m_encodeConfig.inputFormat);
    if (nvStatus != NV_ENC_SUCCESS)
        return false;

    if (m_encodeConfig.preloadedFrameCount >= 2)
    {
        preloadedFrameCount = m_encodeConfig.preloadedFrameCount;
    }

	return true;
}

bool CNvEncoder::processFrame(uint8_t *argb) {
		

		EncodeFrameConfig stEncodeFrame;
		memset(&stEncodeFrame, 0, sizeof(stEncodeFrame));
		stEncodeFrame.rgb = argb;
		stEncodeFrame.width = m_encodeConfig.width;
		stEncodeFrame.height = m_encodeConfig.height;
		stEncodeFrame.stride = m_encodeConfig.width * 4;

		NVENCSTATUS nvStatus = EncodeFrame(&stEncodeFrame, false, m_encodeConfig.width, m_encodeConfig.height);
		if (nvStatus != NV_ENC_SUCCESS)
		{
			return false;
		}
		return true;
}

bool CNvEncoder::FinalizeEncoder() {
	NVENCSTATUS nvStatus = EncodeFrame(NULL, true, m_encodeConfig.width, m_encodeConfig.height);
	if (nvStatus != NV_ENC_SUCCESS)
	{
		return false;
	}

	if (m_encodeConfig.fOutput)
	{
		fclose(m_encodeConfig.fOutput);
	}

	Deinitialize();

	return true;
}
//int main(int argc, char **argv)
//{
//    CNvEncoder nvEncoder;
//    return nvEncoder.EncodeMain(argc, argv);
//}
