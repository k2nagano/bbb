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
typedef unsigned char BYTE;

using namespace std;

HANDLE g_pKeyHandle = 0;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
DWORD g_baudrate = 0;

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
Test(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;
    for (int i = 0; i < N; ++i)
    {
        WORD aNodeId = g_usNodeIds[i];
        WORD aObjectIndex = 0x6076;
        BYTE aObjectSubIndex = 0x0;
        DWORD aNbOfBytesToRead = sizeof(DWORD);
        DWORD aData = 0;
        DWORD aNbOfBytesRead = 0;
        BOOL r = VCS_GetObject(g_pKeyHandle, aNodeId, aObjectIndex, aObjectSubIndex, &aData, aNbOfBytesToRead, &aNbOfBytesRead,
                               p_pErrorCode);
        if (r == 0)
        {
            LogError("VCS_GetObject", aNodeId, r, *p_pErrorCode);
            //return lResult;
	    continue;
        }
        cout << "node" << aNodeId << ": 6076=" << aData << endl;
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

    if ((lResult = Test(&ulErrorCode)) != MMC_SUCCESS)
    {
        LogError("Test", 0, lResult, ulErrorCode);
        return lResult;
    }

    if ((lResult = CloseDevice(&ulErrorCode)) != MMC_SUCCESS)
    {
        LogError("CloseDevice", 0, lResult, ulErrorCode);
        return lResult;
    }

    return lResult;
}
