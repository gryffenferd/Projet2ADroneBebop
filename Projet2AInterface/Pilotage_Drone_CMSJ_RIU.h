/*
    Copyright (C) 2014 Parrot SA

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Parrot nor the names
      of its contributors may be used to endorse or promote products
      derived from this software without specific prior written
      permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.
*/
#ifndef _SDK_EXAMPLE_BD_H_
#define _SDK_EXAMPLE_BD_H_

#include <libARCommands/ARCommands.h>

typedef struct
{
    int flag;
    int roll;
    int pitch;
    int yaw;
    int gaz;
}BD_PCMD_t;

typedef struct READER_THREAD_DATA_t READER_THREAD_DATA_t;

//typedef struct RawFrame_t RawFrame_t; Pour la vidéo donc inutile
typedef struct
{
    ARNETWORKAL_Manager_t *alManager;
    ARNETWORK_Manager_t *netManager;
    ARSTREAM_Reader_t *streamReader;
    ARSAL_Thread_t looperThread;
    ARSAL_Thread_t rxThread;
    ARSAL_Thread_t txThread;
    int d2cPort;
    int c2dPort;

    BD_PCMD_t dataPCMD;

    eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE flyingState;

    ARSAL_Mutex_t mutex;

    ARSAL_Thread_t *readerThreads;
    READER_THREAD_DATA_t *readerThreadsData;
    int run;
} BD_MANAGER_t;

struct READER_THREAD_DATA_t
{
    BD_MANAGER_t *deviceManager;
    int readerBufferId;
};

void *readerRun (void* data);

/** Connection part **/
int ardiscoveryConnect (BD_MANAGER_t *deviceManager);
eARDISCOVERY_ERROR ARDISCOVERY_Connection_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData);
eARDISCOVERY_ERROR ARDISCOVERY_Connection_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData);

/** Network part **/
int startNetwork (BD_MANAGER_t *deviceManager);
void stopNetwork (BD_MANAGER_t *deviceManager);
void onDisconnectNetwork (ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData);

/** send part **/
int sendPCMD(BD_MANAGER_t *deviceManager);
int sendAllStates(BD_MANAGER_t *deviceManager);
int sendDate(BD_MANAGER_t *deviceManager);
int sendAllSettings(BD_MANAGER_t *deviceManager);
int sendTakeoff(BD_MANAGER_t *deviceManager);
int sendLanding(BD_MANAGER_t *deviceManager);
int sendEmergency(BD_MANAGER_t *deviceManager);
eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause);

/** Commands Part **/
void *keypad(BD_MANAGER_t *deviceManager);
void *joystick(BD_MANAGER_t *deviceManager);

/** Commands callback part **/
void registerARCommandsCallbacks (BD_MANAGER_t *deviceManager);
void unregisterARCommandsCallbacks();
void batteryStateChangedCallback (uint8_t percent, void *custom);
void DateCallback(char *date, void *custom);
void AttitudeCallback(float roll, float pitch, float yaw, void *custom);
void AltitudeCallback(double altitude, void *custom);
void SpeedCallback(float speedX, float speedY, float speedZ, void *custom);

void control(int argc, char *argv[]);
#endif /* _SDK_EXAMPLE_BD_H_ */
