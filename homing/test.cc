#include "Definitions.h"
#include <getopt.h>
#include <iostream>
#include <list>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

typedef void* HANDLE;
typedef int BOOL;
typedef unsigned short WORD;
typedef unsigned int DWORD;
// typedef unsigned long long DWORD64;
typedef unsigned char BYTE;

using namespace std;

HANDLE g_pKeyHandle = 0;
WORD g_usNodeId = 1;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
DWORD g_baudrate = 0;
int g_lHomeOffset = 0;
signed char g_scHomingMethod = 37;
unsigned int g_ulHomingAcceleration = 1000;
unsigned int g_ulSpeedSwitch = 100;
unsigned int g_ulSpeedIndex = 10;
unsigned short g_usCurrentThreshold = 500;
unsigned int g_resolution = 131072;

const int N = 2;
WORD g_usNodeIds[N] = {1, 2};

#ifndef MMC_SUCCESS
#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
#define MMC_FAILED 1
#endif

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void LogInfo(string message);
void PrintUsage();
void PrintSettings();
int OpenDevice(unsigned int* p_pErrorCode);
int CloseDevice(unsigned int* p_pErrorCode);
void SetDefaultParameters();
int ParseArguments(int argc, char** argv);
int Test(unsigned int* p_pErrorCode);

void
PrintUsage()
{
    cout << "Usage: test" << endl;
    cout << "\t-h : this help" << endl;
    cout << "\t-n : node id (default 1)" << endl;
    cout << "\t-d   : device name (EPOS2, EPOS4, default - EPOS4)" << endl;
    cout << "\t-s   : protocol stack name (MAXON_RS232, CANopen, MAXON SERIAL V2, default - CANopen)" << endl;
    cout << "\t-i   : interface name (RS232, USB, CAN_ixx_usb 0, CAN_kvaser_usb 0,... default - CAN_mcp251x 0)" << endl;
    cout << "\t-p   : port name (COM1, USB0, CAN0,... default - CAN0)" << endl;
    cout << "\t-b   : baudrate (115200, 1000000,... default - 1000000)" << endl;
}

void
LogError(string functionName, WORD aNodeId, int p_lResult, unsigned int p_ulErrorCode)
{
    // cerr << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")" << endl;

    char error[64];
    int r = VCS_GetErrorInfo(p_ulErrorCode, error, sizeof(error));
    stringstream err;
    string node;
    if ((aNodeId == 1) || (aNodeId == 2))
    {
        node = "[nodeId=" + to_string(aNodeId) + "]";
    }
    err << functionName << " failed " << node << "(result=" << p_lResult << ", errorCode=0x" << hex << p_ulErrorCode << " `"
        << error << "')";
    cerr << err.str() << endl;
}

void
LogInfo(string message)
{
    cout << message << endl;
}

void
SeparatorLine()
{
    const int lineLength = 65;
    for (int i = 0; i < lineLength; i++)
    {
        cout << "-";
    }
    cout << endl;
}

void
PrintSettings()
{
    stringstream msg;

    msg << "default settings:" << endl;
    msg << "node id             = " << g_usNodeId << endl;
    msg << "device name         = '" << g_deviceName << "'" << endl;
    msg << "protocal stack name = '" << g_protocolStackName << "'" << endl;
    msg << "interface name      = '" << g_interfaceName << "'" << endl;
    msg << "port name           = '" << g_portName << "'" << endl;
    msg << "baudrate            = " << g_baudrate;

    LogInfo(msg.str());

    SeparatorLine();
}

void
SetDefaultParameters()
{
    // USB
    g_usNodeId = 2;
    g_deviceName = "EPOS4";
    // g_protocolStackName = "MAXON SERIAL V2";
    // g_interfaceName = "USB";
    // g_portName = "USB0";
    // g_baudrate = 1000000;
#if defined(FOR_RASPI)
    // https://qiita.com/jamjam/items/be97d25e90c7aa9b92c5
    g_protocolStackName = "CANopen";
    g_interfaceName = "CAN_mcp251x 0";
    g_portName = "CAN0";
#else  // defined(FOR_RASPI)
    g_protocolStackName = "CANopen";
    g_interfaceName = "CAN_peak_usb 0";
    g_portName = "CAN0";
#endif // defined(FOR_RASPI)
    g_baudrate = 1000000;
}

int
OpenDevice(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;

    char* pDeviceName = new char[255];
    char* pProtocolStackName = new char[255];
    char* pInterfaceName = new char[255];
    char* pPortName = new char[255];

    strcpy(pDeviceName, g_deviceName.c_str());
    strcpy(pProtocolStackName, g_protocolStackName.c_str());
    strcpy(pInterfaceName, g_interfaceName.c_str());
    strcpy(pPortName, g_portName.c_str());

    LogInfo("Open device...");

    g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

    if (g_pKeyHandle != 0 && *p_pErrorCode == 0)
    {
        unsigned int lBaudrate = 0;
        unsigned int lTimeout = 0;

        if (VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode) != 0)
        {
            if (VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode) != 0)
            {
                if (VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode) != 0)
                {
                    if (g_baudrate == (int)lBaudrate)
                    {
                        lResult = MMC_SUCCESS;
                    }
                }
            }
        }
    }
    else
    {
        g_pKeyHandle = 0;
    }

    delete[] pDeviceName;
    delete[] pProtocolStackName;
    delete[] pInterfaceName;
    delete[] pPortName;

    return lResult;
}

int
CloseDevice(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;

    *p_pErrorCode = 0;

    LogInfo("Close device");

    if (VCS_CloseDevice(g_pKeyHandle, p_pErrorCode) != 0 && *p_pErrorCode == 0)
    {
        lResult = MMC_SUCCESS;
    }

    return lResult;
}

int
ParseArguments(int argc, char** argv)
{
    int lOption;
    int lResult = MMC_SUCCESS;

    opterr = 0;

    while ((lOption = getopt(argc, argv, "hb:n:")) != -1)
    {
        switch (lOption)
        {
        case 'h':
            PrintUsage();
            lResult = 1;
            break;
        case 'b':
            g_baudrate = atoi(optarg);
            break;
        case 'n':
            g_usNodeId = (unsigned short)atoi(optarg);
            break;
        case '?': // unknown option...
            stringstream msg;
            msg << "Unknown option: '" << char(optopt) << "'!";
            LogInfo(msg.str());
            PrintUsage();
            lResult = MMC_FAILED;
            break;
        }
    }

    return lResult;
}
int
Prepare(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;
    {

    }
    lResult = MMC_SUCCESS;
    return lResult;
}
int
Test(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;
    int aHomePosDeg = 8260;
    for (int i = 0; i < N; ++i)
    {
        WORD aNodeId = g_usNodeIds[i];
        int aHomePos = static_cast<int>(aHomePosDeg * g_resolution / 360);
        int r = 0;
        r = VCS_ActivateHomingMode(g_pKeyHandle, aNodeId, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_ActivateHomingMode", aNodeId, r, *p_pErrorCode);
            return lResult;
        }
        r = VCS_SetHomingParameter(g_pKeyHandle, aNodeId, g_ulHomingAcceleration, g_ulSpeedSwitch, g_ulSpeedIndex,
                                   g_lHomeOffset, g_usCurrentThreshold, aHomePos, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_SetHomingParameter", aNodeId, r, *p_pErrorCode);
            return lResult;
        }
        r = VCS_FindHome(g_pKeyHandle, aNodeId, g_scHomingMethod, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_FindHome", aNodeId, r, *p_pErrorCode);
            return lResult;
        }
        for (int i = 0; i < 10; ++i)
        {
            r = VCS_WaitForHomingAttained(g_pKeyHandle, aNodeId, 1000, p_pErrorCode);
            if (!r)
            {
                LogError("VCS_WaitForHomingAttained", aNodeId, r, *p_pErrorCode);
                return lResult;
            }
            int homing_attained = 0;
            int homing_error = 0;
            r = VCS_GetHomingState(g_pKeyHandle, aNodeId, &homing_attained, &homing_error, p_pErrorCode);
            if (!r)
            {
                LogError("VCS_GetHomingState", aNodeId, r, *p_pErrorCode);
                return lResult;
            }
            cout << "homing_attained = " << homing_attained << endl;
            cout << "homing_error = " << homing_error << endl;
            if (homing_attained)
            {
                break;
            }
        }
        r = VCS_StopHoming(g_pKeyHandle, aNodeId, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_StopHoming", aNodeId, r, *p_pErrorCode);
            return lResult;
        }
        int position = 0;
        r = VCS_GetPositionIs(g_pKeyHandle, aNodeId, &position, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_GetPositionIs", aNodeId, r, *p_pErrorCode);
            return lResult;
        }
        cout << "position = " << position << endl;
    }
    lResult = MMC_SUCCESS;
    return lResult;
}
int
Test2(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;
    for (int i = 0; i < N; ++i)
    {
        WORD aNodeId = g_usNodeIds[i];
        int r = 0;
        r = VCS_SetEnableState(g_pKeyHandle, aNodeId, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_SetEnableState", aNodeId, r, *p_pErrorCode);
            return lResult;
        }
        r = VCS_ActivateProfilePositionMode(g_pKeyHandle, aNodeId, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_ActivateProfilePositionMode", aNodeId, r, *p_pErrorCode);
            return lResult;
        }
        long aTargetPosition = 0;
        BOOL aAbsolute = 1;
        BOOL aImmediately = 1;
        r = VCS_MoveToPosition(g_pKeyHandle, aNodeId, aTargetPosition, aAbsolute, aImmediately, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_MoveToPosition", aNodeId, r, *p_pErrorCode);
            return lResult;
        }
        sleep(1);
        sleep(1);
        r = VCS_HaltPositionMovement(g_pKeyHandle, aNodeId, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_HaltPositionMovement", aNodeId, r, *p_pErrorCode);
            return lResult;
        }

        r = VCS_SetDisableState(g_pKeyHandle, aNodeId, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_SetDisableState", aNodeId, r, *p_pErrorCode);
            return lResult;
        }
        int position = 0;
        r = VCS_GetPositionIs(g_pKeyHandle, aNodeId, &position, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_GetPositionIs", aNodeId, r, *p_pErrorCode);
            return lResult;
        }
        cout << "position = " << position << endl;
    }
    lResult = MMC_SUCCESS;
    return lResult;
}
int
Test3(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;
    for (int i = 0; i < N; ++i)
    {
        WORD aNodeId = g_usNodeIds[i];
        BOOL r = 0;

        // WORD gain = EC_PID_POSITION_CONTROLLER;
        // WORD controller = EG_PIDPC_P_GAIN;
        // DWORD64 value = 0;
        // r = VCS_GetControllerGain(g_pKeyHandle, aNodeId, controller, gain, &value, p_pErrorCode);
        // if (!r)
        // {
        //     LogError("VCS_GetControllerGain", aNodeId, r, *p_pErrorCode);
        //     return lResult;
        // }
        // cout << "value = " << value << endl;
        DWORD aMaxFollowingError = 200000;
        // r = VCS_SetMaxFollowingError(g_pKeyHandle, aNodeId, aMaxFollowingError, p_pErrorCode);
        // if (!r)
        // {
        //     LogError("VCS_SetMaxFollowingError", aNodeId, r, *p_pErrorCode);
        //     return lResult;
        // }
        r = VCS_GetMaxFollowingError(g_pKeyHandle, aNodeId, &aMaxFollowingError, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_GetMaxFollowingError", aNodeId, r, *p_pErrorCode);
            return lResult;
        }
        cout << "aMaxFollowingError = " << aMaxFollowingError << endl;

        DWORD aEncoderResolution = g_resolution;
        BOOL aInvertedPolarity = 0;
        // r = VCS_SetIncEncoderParameter(g_pKeyHandle, aNodeId, aEncoderResolution, aInvertedPolarity, p_pErrorCode);
        // if (!r)
        // {
        //     LogError("VCS_SetIncEncoderParameter", aNodeId, r, *p_pErrorCode);
        //     return lResult;
        // }
        r = VCS_GetIncEncoderParameter(g_pKeyHandle, aNodeId, &aEncoderResolution, &aInvertedPolarity, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_GetIncEncoderParameter", aNodeId, r, *p_pErrorCode);
            return lResult;
        }
        cout << "aEncoderResolution = " << aEncoderResolution << endl;
        cout << "aInvertedPolarity = " << aInvertedPolarity << endl;

        r = VCS_GetHallSensorParameter(g_pKeyHandle, aNodeId, &aInvertedPolarity, p_pErrorCode);
        if (!r)
        {
            LogError("VCS_GetHallSensorParameter", aNodeId, r, *p_pErrorCode);
            return lResult;
        }
        cout << "aInvertedPolarity = " << aInvertedPolarity << endl;
    }
    lResult = MMC_SUCCESS;
    return lResult;
}
int
main(int argc, char** argv)
{
    int lResult = MMC_FAILED;
    unsigned int ulErrorCode = 0;

    SetDefaultParameters();

    if ((lResult = ParseArguments(argc, argv)) != MMC_SUCCESS)
    {
        return lResult;
    }

    PrintSettings();

    if ((lResult = OpenDevice(&ulErrorCode)) != MMC_SUCCESS)
    {
        LogError("OpenDevice", 0, lResult, ulErrorCode);
        return lResult;
    }

    // if ((lResult = Test(&ulErrorCode)) != MMC_SUCCESS)
    // {
    //     LogError("Test", 0, lResult, ulErrorCode);
    //     return lResult;
    // }

    if ((lResult = Prepare(&ulErrorCode)) != MMC_SUCCESS)
    {
        LogError("Prepare", 0, lResult, ulErrorCode);
        return lResult;
    }

    if ((lResult = CloseDevice(&ulErrorCode)) != MMC_SUCCESS)
    {
        LogError("CloseDevice", 0, lResult, ulErrorCode);
        return lResult;
    }

    return lResult;
}
