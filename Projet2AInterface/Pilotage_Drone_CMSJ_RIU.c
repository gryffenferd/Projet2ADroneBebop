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
/**
 * @file BebopDroneDecodeStream.c
 * @brief This file contains sources about basic arsdk example decoding video stream from a BebopDrone with ffmpeg
 * @date 08/01/2015
 */

/*****************************************
 *
 *             include file :
 *
 *****************************************/
#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <SDL/SDL.h>
#include <pthread.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>

#include <libARSAL/ARSAL.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARNetwork/ARNetwork.h>
#include <libARNetworkAL/ARNetworkAL.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARStream/ARStream.h>

#include "ihm.h"
#include <xdrvlib.h>
#include "Pilotage_Drone_CMSJ_RIU.h"

/*****************************************
 *
 *             define :readerRun
 *
 *****************************************/
#define TAG "Pilotage Drone CMSJ RIU"
#define BD_IP_ADDRESS "192.168.42.1"
#define BD_DISCOVERY_PORT 44444
#define BD_C2D_PORT 54321 // should be read from Json
#define BD_D2C_PORT 43210

#define BD_NET_CD_NONACK_ID 10
#define BD_NET_CD_ACK_ID 11
#define BD_NET_CD_EMERGENCY_ID 12
#define BD_NET_CD_VIDEO_ACK_ID 13
#define BD_NET_DC_NAVDATA_ID 127
#define BD_NET_DC_EVENT_ID 126
#define BD_NET_DC_VIDEO_DATA_ID 125

#define BD_NET_DC_VIDEO_FRAG_SIZE 1000
#define BD_NET_DC_VIDEO_MAX_NUMBER_OF_FRAG 128

#define BD_RAW_FRAME_BUFFER_SIZE 50
#define BD_RAW_FRAME_POOL_SIZE 50

#define ERROR_STR_LENGTH 2048

#define MUTEX 8

/*****************************************
 *                     					 *
 *             implementation :		     *
 *					                     *
 *****************************************/
extern int controler;
extern float value;
extern float sx;
extern float sy;
extern float sz;
extern float altitud;
extern float rollValue;
extern float pitchValue;
extern int state;
extern int controler;
extern GMutex *mutex[MUTEX];

extern PCMD_t PCMD;
extern int keyEvent;

static ARNETWORK_IOBufferParam_t c2dParams[] = {
    /* Non-acknowledged commands. */
    {
        .ID = BD_NET_CD_NONACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 2,
        .dataCopyMaxSize = 128,
        .isOverwriting = 1,
    },
    /* Acknowledged commands. */
    {
        .ID = BD_NET_CD_ACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = 500,
        .numberOfRetry = 3,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    /* Emergency commands. */
    {
        .ID = BD_NET_CD_EMERGENCY_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 10,
        .ackTimeoutMs = 100,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 1,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
};
static const size_t numC2dParams = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);

static ARNETWORK_IOBufferParam_t d2cParams[] = {
    {
        .ID = BD_NET_DC_NAVDATA_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    {
        .ID = BD_NET_DC_EVENT_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = 500,
        .numberOfRetry = 3,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
};
static const size_t numD2cParams = sizeof(d2cParams) / sizeof(ARNETWORK_IOBufferParam_t);

static int commandBufferIds[] = {
    BD_NET_DC_NAVDATA_ID,
    BD_NET_DC_EVENT_ID,
};
static const size_t numOfCommandBufferIds = sizeof(commandBufferIds) / sizeof(int);

int continuer = 1;
char gErrorStr[ERROR_STR_LENGTH];

// reader thread
void *readerRun (void* data)
{
    BD_MANAGER_t *deviceManager = NULL;
    int bufferId = 0;
    int failed = 0;

    // Allocate some space for incoming data.
    const size_t maxLength = 128 * 1024;
    void *readData = malloc (maxLength);
    if (readData == NULL)
    {
        failed = 1;
    }

    if (!failed)
    {
        // get thread data.
        if (data != NULL)
        {
            bufferId = ((READER_THREAD_DATA_t *)data)->readerBufferId;
            deviceManager = ((READER_THREAD_DATA_t *)data)->deviceManager;

            if (deviceManager == NULL)
            {
                failed = 1;
            }
        }
        else
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        while (deviceManager->run)
        {
            eARNETWORK_ERROR netError = ARNETWORK_OK;
            int length = 0;
            int skip = 0;

            // read data
            netError = ARNETWORK_Manager_ReadDataWithTimeout (deviceManager->netManager, bufferId, readData, maxLength, &length, 1000);
            if (netError != ARNETWORK_OK)
            {
                if (netError != ARNETWORK_ERROR_BUFFER_EMPTY)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNETWORK_Manager_ReadDataWithTimeout () failed : %s", ARNETWORK_Error_ToString(netError));
                }
                skip = 1;
            }

            if (!skip)
            {
                // Forward data to the CommandsManager
                eARCOMMANDS_DECODER_ERROR cmdError = ARCOMMANDS_DECODER_OK;
                cmdError = ARCOMMANDS_Decoder_DecodeBuffer ((uint8_t *)readData, length);
                if ((cmdError != ARCOMMANDS_DECODER_OK) && (cmdError != ARCOMMANDS_DECODER_ERROR_NO_CALLBACK))
                {
                    char msg[128];
                    ARCOMMANDS_Decoder_DescribeBuffer ((uint8_t *)readData, length, msg, sizeof(msg));
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARCOMMANDS_Decoder_DecodeBuffer () failed : %d %s", cmdError, msg);
                }
            }
        }
    }

    if (readData != NULL)
    {
        free (readData);
        readData = NULL;
    }

    return NULL;
}


/***********************************************************
 *					 		                               *
 *            		Connexion Part		 	               *
 *					  		                               *
 **********************************************************/
int ardiscoveryConnect (BD_MANAGER_t *deviceManager)
{
    int failed = 0;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- ARDiscovery Connection");

    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;


    ARDISCOVERY_Connection_ConnectionData_t *discoveryData = ARDISCOVERY_Connection_New (ARDISCOVERY_Connection_SendJsonCallback, ARDISCOVERY_Connection_ReceiveJsonCallback, deviceManager, &err);
    if (discoveryData == NULL || err != ARDISCOVERY_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while creating discoveryData : %s", ARDISCOVERY_Error_ToString(err));
        failed = 1;
    }

    if (!failed)
    {
        eARDISCOVERY_ERROR err = ARDISCOVERY_Connection_ControllerConnection(discoveryData, BD_DISCOVERY_PORT, BD_IP_ADDRESS);
        if (err != ARDISCOVERY_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while opening discovery connection : %s", ARDISCOVERY_Error_ToString(err));
            failed = 1;
        }
    }

    ARDISCOVERY_Connection_Delete(&discoveryData);

    return failed;
}

eARDISCOVERY_ERROR ARDISCOVERY_Connection_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    if ((dataTx != NULL) && (dataTxSize != NULL) && (deviceManager != NULL))
    {
        *dataTxSize = sprintf((char *)dataTx, "{ \"%s\": %d,\n \"%s\": \"%s\",\n \"%s\": \"%s\" }",
                              ARDISCOVERY_CONNECTION_JSON_D2CPORT_KEY, deviceManager->d2cPort,
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY, "toto",
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY, "tata") + 1;
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }

    return err;
}

eARDISCOVERY_ERROR ARDISCOVERY_Connection_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    if ((dataRx != NULL) && (dataRxSize != 0) && (deviceManager != NULL))
    {
        char *json = malloc(dataRxSize + 1);
        strncpy(json, (char *)dataRx, dataRxSize);
        json[dataRxSize] = '\0';

        //read c2dPort ...

        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "    - ReceiveJson:%s ", json);

        free(json);
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }

    return err;
}

/***********************************************************
 *					 		                               *
 *            		Network Part		 	               *
 *					  		                               *
 **********************************************************/
int startNetwork (BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    eARNETWORK_ERROR netError = ARNETWORK_OK;
    eARNETWORKAL_ERROR netAlError = ARNETWORKAL_OK;
    int pingDelay = 0; // 0 means default, -1 means no ping

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Start ARNetwork");

    // Create the ARNetworkALManager
    deviceManager->alManager = ARNETWORKAL_Manager_New(&netAlError);
    if (netAlError != ARNETWORKAL_OK)
    {
        failed = 1;
    }

    if (!failed)
    {
        // Initilize the ARNetworkALManager
        netAlError = ARNETWORKAL_Manager_InitWifiNetwork(deviceManager->alManager, BD_IP_ADDRESS, BD_C2D_PORT, BD_D2C_PORT, 1);
        if (netAlError != ARNETWORKAL_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create the ARNetworkManager.
        deviceManager->netManager = ARNETWORK_Manager_New(deviceManager->alManager, numC2dParams, c2dParams, numD2cParams, d2cParams, pingDelay, onDisconnectNetwork, deviceManager, &netError);
        if (netError != ARNETWORK_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create and start Tx and Rx threads.
        if (ARSAL_Thread_Create(&(deviceManager->rxThread), ARNETWORK_Manager_ReceivingThreadRun, deviceManager->netManager) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of Rx thread failed.");
            failed = 1;
        }

        if (ARSAL_Thread_Create(&(deviceManager->txThread), ARNETWORK_Manager_SendingThreadRun, deviceManager->netManager) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of Tx thread failed.");
            failed = 1;
        }
    }

    // Print net error
    if (failed)
    {
        if (netAlError != ARNETWORKAL_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNetWorkAL Error : %s", ARNETWORKAL_Error_ToString(netAlError));
        }

        if (netError != ARNETWORK_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNetWork Error : %s", ARNETWORK_Error_ToString(netError));
        }
    }

    return failed;
}

void stopNetwork (BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    eARNETWORK_ERROR netError = ARNETWORK_OK;
    eARNETWORKAL_ERROR netAlError = ARNETWORKAL_OK;
    int pingDelay = 0; // 0 means default, -1 means no ping

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Stop ARNetwork");

    // ARNetwork cleanup
    if (deviceManager->netManager != NULL)
    {
        ARNETWORK_Manager_Stop(deviceManager->netManager);
        if (deviceManager->rxThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->rxThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->rxThread));
            deviceManager->rxThread = NULL;
        }

        if (deviceManager->txThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->txThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->txThread));
            deviceManager->txThread = NULL;
        }
    }

    if (deviceManager->alManager != NULL)
    {
        ARNETWORKAL_Manager_Unlock(deviceManager->alManager);

        ARNETWORKAL_Manager_CloseWifiNetwork(deviceManager->alManager);
    }

    ARNETWORK_Manager_Delete(&(deviceManager->netManager));
    ARNETWORKAL_Manager_Delete(&(deviceManager->alManager));
}

void onDisconnectNetwork (ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "onDisconnectNetwork ...");
    //gIHMRun = 0;
}

/***********************************************************
 *					 		                               *
 *            	    	Send Part		 	               *
 *					  		                               *
 **********************************************************/
int sendPCMD(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    // Send pcmd command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingPCMD(cmdBuffer, sizeof(cmdBuffer), &cmdSize, (uint8_t)deviceManager->dataPCMD.flag, (uint8_t)deviceManager->dataPCMD.roll, deviceManager->dataPCMD.pitch, (uint8_t)deviceManager->dataPCMD.yaw, (uint8_t)deviceManager->dataPCMD.gaz, 0);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        // The commands sent in loop should be sent to a buffer not acknowledged ; here BD_NET_CD_NONACK_ID
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_NONACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        sentStatus = 0;
    }

    return sentStatus;
}

int sendAllStates(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send get all states");

    // Send get all states command
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonAllStates(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send get all states command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}

int sendDate(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send date");

    // Send date command
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentDate(cmdBuffer, sizeof(cmdBuffer), &cmdSize, "2015-04-20");
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send Streaming command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    if (sentStatus)
    {
        // Send time command
        cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentTime(cmdBuffer, sizeof(cmdBuffer), &cmdSize, "'T'101533+0200");
        if (cmdError == ARCOMMANDS_GENERATOR_OK)
        {
            netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
        }

        if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
        {
            ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send Streaming command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
            sentStatus = 0;
        }
    }

    return sentStatus;
}

int sendAllSettings(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send get all settings");

    // Send get all settings command
    cmdError = ARCOMMANDS_Generator_GenerateCommonSettingsAllSettings(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send get all settings command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;

}

int sendTakeoff(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send take off");

    // Send take off command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingTakeOff(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send takeoff command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}

int sendLanding(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send landing");

    // Send landing command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingLanding(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send landing command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}

int sendEmergency(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send Emergency");

    // Send emergency command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingEmergency(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_EMERGENCY_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send emergency command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}

eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause)
{
    eARNETWORK_MANAGER_CALLBACK_RETURN retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DEFAULT;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "    - arnetworkCmdCallback %d, cause:%d ", buffer_id, cause);

    if (cause == ARNETWORK_MANAGER_CALLBACK_STATUS_TIMEOUT)
    {
        retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DATA_POP;
    }

    return retval;
}

/***********************************************************
 *					 		                               *
 *	            	Commands Part		 	               *
 *					  		                               *
 **********************************************************/
void *joystick(BD_MANAGER_t *deviceManager)
{
	int i;
	SDL_Event event;

	SDL_Init(SDL_INIT_VIDEO);
	SDL_Init(SDL_INIT_JOYSTICK);

	SDL_Joystick *joystick;
	joystick = SDL_JoystickOpen(0);

	SDL_JoystickEventState(SDL_ENABLE);
	while(continuer)
	{
		SDL_WaitEvent(&event);
		switch(event.type)
		{
			case SDL_JOYBUTTONUP:
				deviceManager->dataPCMD.roll = 0;
    			deviceManager->dataPCMD.pitch = 0;
		    	deviceManager->dataPCMD.yaw = 0;
				deviceManager->dataPCMD.gaz = 0;
				break;

			case SDL_JOYBUTTONDOWN:
				switch(event.jbutton.button)
				{
					case 0:
						deviceManager->dataPCMD.gaz = 100;
						break;
					case 1:
						sendLanding(deviceManager);
						deviceManager->dataPCMD.roll = 0;
    				    deviceManager->dataPCMD.pitch = 0;
		    		    deviceManager->dataPCMD.yaw = 0;
						deviceManager->dataPCMD.gaz = 0;
						break;
					case 2:
						deviceManager->dataPCMD.gaz = -100;
						break;
					case 3:
						deviceManager->dataPCMD.gaz = -100;
						break;
					case 4:
						deviceManager->dataPCMD.yaw = -50;
						break;
					case 5:
						deviceManager->dataPCMD.yaw = 50;
						break;
					case 6:
						continuer = 0;
						break;
					case 7:
						sendTakeoff(deviceManager);
						deviceManager->dataPCMD.flag = 1;
						break;
				}
			break;

			/* Tests pour savoir quel axe est utilisé
			 * le test de > 5000 et -5000 permet d'avoir une zone morte à cause des dualshock */

			case SDL_JOYAXISMOTION:
				if(event.jaxis.axis == 0)			// si joystick droite/gauche
				{

					if(event.jaxis.value < -1000)
						deviceManager->dataPCMD.roll = event.jaxis.value/330;
					if(event.jaxis.value > 1000)
						deviceManager->dataPCMD.roll = event.jaxis.value/330;
					if(event.jaxis.value >= -1000 && event.jaxis.value <= 1000)
						deviceManager->dataPCMD.roll = 0;
				}
				if(event.jaxis.axis == 1)			// si joytsick haut/bas
				{
					if(event.jaxis.value < -1000)
						deviceManager->dataPCMD.pitch = event.jaxis.value/(-330);
					if(event.jaxis.value > 1000)
						deviceManager->dataPCMD.pitch = event.jaxis.value/(-330);
					if(event.jaxis.value >= -1000 && event.jaxis.value <= 1000)
						deviceManager->dataPCMD.pitch = 0;
				}

				break;

		}
	}

	SDL_JoystickClose(joystick);
	SDL_Quit();
}

void *joystick2(BD_MANAGER_t *deviceManager)
{
	int i;
	SDL_Event event;

	SDL_Init(SDL_INIT_VIDEO);
	SDL_Init(SDL_INIT_JOYSTICK);

	SDL_Joystick *joystick;
	joystick = SDL_JoystickOpen(0);

	SDL_JoystickEventState(SDL_ENABLE);

	while(continuer)
	{
		SDL_WaitEvent(&event);
		switch(event.type)
		{
			case SDL_JOYBUTTONUP:
				deviceManager->dataPCMD.gaz = 0;
				break;

			case SDL_JOYBUTTONDOWN:
				switch(event.jbutton.button)
				{
					case 0:
						sendLanding(deviceManager);
						break;
					case 1:
						sendTakeoff(deviceManager);
						deviceManager->dataPCMD.flag = 1;
						break;
					case 2:
						deviceManager->dataPCMD.gaz = 100;
						break;
					case 3:
						deviceManager->dataPCMD.gaz = -100;
						break;
				}
			break;

			/* Tests pour savoir quel axe est utilisé
			 * le test de > 5000 et -5000 permet d'avoir une zone morte à cause des dualshock */

			case SDL_JOYAXISMOTION:
				if(event.jaxis.axis == 0)			// si joystick droite/gauche
				{
					if(event.jaxis.value < -500)
						deviceManager->dataPCMD.roll = event.jaxis.value/330;
					if(event.jaxis.value > 500)
						deviceManager->dataPCMD.roll = event.jaxis.value/330;
					if(event.jaxis.value >= -500 && event.jaxis.value <= 500)
						deviceManager->dataPCMD.roll = 0;
				}
				if(event.jaxis.axis == 1)			// si joytsick haut/bas
				{
					if(event.jaxis.value < -500)
						deviceManager->dataPCMD.pitch = event.jaxis.value/(-330);
					if(event.jaxis.value > 500)
						deviceManager->dataPCMD.pitch = event.jaxis.value/(-330);
					if(event.jaxis.value >= -500 && event.jaxis.value <= 500)
						deviceManager->dataPCMD.pitch = 0;
				}
				if(event.jaxis.axis == 2 && event.jaxis.value > 0)
				{
					continuer = 0;
				}
				if(event.jaxis.axis == 3)			// si joystick rotation
				{
					if(event.jaxis.value < -500)
						deviceManager->dataPCMD.yaw = event.jaxis.value/400;
					if(event.jaxis.value > 500)
						deviceManager->dataPCMD.yaw = event.jaxis.value/400;
					if(event.jaxis.value >= -500 && event.jaxis.value <= 500)
						deviceManager->dataPCMD.yaw = 0;
				}
				break;
		}
	}

	SDL_JoystickClose(joystick);
	SDL_Quit();
}

/***********************************************************
 *					 		                               *
 *            		Commands Callback Part	 	           *
 *					  		                               *
 **********************************************************/
void registerARCommandsCallbacks (BD_MANAGER_t *deviceManager)
{
    ARCOMMANDS_Decoder_SetCommonCommonStateBatteryStateChangedCallback(batteryStateChangedCallback, deviceManager);
    ARCOMMANDS_Decoder_SetCommonCommonStateCurrentDateChangedCallback(DateCallback,deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAttitudeChangedCallback(AttitudeCallback,deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAltitudeChangedCallback(AltitudeCallback,deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateSpeedChangedCallback(SpeedCallback,deviceManager);
}

void unregisterARCommandsCallbacks ()
{
    ARCOMMANDS_Decoder_SetCommonCommonStateBatteryStateChangedCallback (NULL, NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateFlyingStateChangedCallback(NULL, NULL);
}

/**
* Connaitre le niveau de la batterie
**/
void batteryStateChangedCallback (uint8_t percent, void *custom)
{
    // callback of changing of battery level
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;

    g_mutex_lock(mutex[BATTERY]);
    if (deviceManager != NULL)
    {
        value = (float)percent;
    }
    g_mutex_unlock(mutex[BATTERY]);
}

/**
* Connaitre la date
**/
void DateCallback(char *date, void *custom)
{
   BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;

   if ((deviceManager != NULL))
   {
      ARSAL_PRINT(ARSAL_PRINT_INFO,TAG,"Date : %s",date);

   }

   else{

      ARSAL_PRINT(ARSAL_PRINT_ERROR,TAG,"Date error");

   }
}

/**
* Connaitre le pitch, roll et yaw du drone
**/
void AttitudeCallback(float roll, float pitch, float yaw, void *custom)
{

    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;

    g_mutex_lock(mutex[ATTITUDE]);

    if ((deviceManager != NULL))
    {
		rollValue = roll;
		pitchValue = pitch;
    }
    g_mutex_unlock(mutex[ATTITUDE]);

}

/**
* Connaitre l'altitude
**/
void AltitudeCallback(double altitude, void *custom)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;

    g_mutex_lock(mutex[ALTITUDE]);
    if ((deviceManager != NULL))
    {
        altitud = (float)altitude;
    }
    g_mutex_unlock(mutex[ALTITUDE]);

}

/**
* Connaitre la vitesse (x, y, z)
**/
void SpeedCallback(float speedX, float speedY, float speedZ, void *custom)
{
    g_mutex_lock(mutex[SPEED]);
    sx = speedX;
    sy = speedY;
    sz = speedZ;
    g_mutex_unlock(mutex[SPEED]);
}

void *mouse3D(mouse_t *mouse)
{
Display *display;
Window root, window;

int screennumber,width,height;
XSizeHints *sizehints;
XWMHints *wmhints;
XClassHint *classhints;

char *WinName = "Magellan 3D Controller";
XTextProperty WindowName;

XEvent report;
MagellanFloatEvent MagellanEvent;

XComposeStatus compose;
KeySym keysym;

char MagellanBuffer[ 256 ];

float z=0; // y [-400 ; 400]
float r=0; // b [-400 ; 400]
float y=0; // c [-400 ; 400]
float x=0; // a [-350 ; 350]

/****************** Open a Window ******************************************/
 sizehints  = XAllocSizeHints();
 wmhints    = XAllocWMHints();
 classhints = XAllocClassHint();

 if ( (sizehints==NULL) || (wmhints==NULL) || (classhints==NULL) )
  {
   fprintf( stderr, "Can't allocate memory! Exit ... \n" );
   exit( -1 );
  }

 display = XOpenDisplay( NULL );
 if ( display == NULL )
  {
   fprintf( stderr, "Can't open display! Exit ... \n");
   exit( -1 );
  }

 screennumber = DefaultScreen(display);
 width  = DisplayWidth(display,screennumber);
 height = DisplayHeight(display,screennumber);
 root   = DefaultRootWindow(display);
 window = XCreateSimpleWindow( display, root, 0,0, width/5*3,height/8, 20,
			       BlackPixel(display,screennumber),
			       WhitePixel(display,screennumber) );

 XStringListToTextProperty( &WinName, 1, &WindowName );

 XSetWMProperties( display, window, &WindowName, NULL, mouse->argv,
        mouse->argc, sizehints, wmhints, classhints );

 /************************* Create 3D Event Types ***************************/
 if ( !MagellanInit( display, window ) )
  {
   fprintf( stderr, "No driver is running. Exit ... \n" );
   exit(-1);
  }
 /************************* Main Loop ***************************************/
 XSelectInput( display, window, KeyPressMask | KeyReleaseMask );

 while(continuer)
  {
   XNextEvent( display, &report );
   switch( report.type )
    {
     case ClientMessage :
      switch( MagellanTranslateEvent( display, &report, &MagellanEvent, 1.0, 1.0 ) )
       {
        case MagellanInputMotionEvent :
        	MagellanRemoveMotionEvents( display );
			z = MagellanEvent.MagellanData[ MagellanY ];
			r = MagellanEvent.MagellanData[ MagellanB ];
			y = MagellanEvent.MagellanData[ MagellanC ];
			x = MagellanEvent.MagellanData[ MagellanA ];

            mouse->deviceManager->dataPCMD.pitch = (-1)*(x/4);
            mouse->deviceManager->dataPCMD.roll = y/4.5;
            mouse->deviceManager->dataPCMD.yaw = (-1)*(r/4.5);
            mouse->deviceManager->dataPCMD.gaz = z/4.5;

 		break;
       }
      break;
     }
  }
 MagellanClose( display );
 }

/***********************************************************
 *					 		   							   *
 *            		    Main		 	   				   *
 *					  		   							   *
 **********************************************************/

gpointer drone(void* data)
{

    ihm_t *data3 = (ihm_t*)data;

    /* local declarations */
    int failed = 0;

    BD_MANAGER_t *deviceManager = malloc(sizeof(BD_MANAGER_t));

    if (deviceManager == NULL)
    {
        failed = 1;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "deviceManager alloc error !");
    }

    if (!failed)
    {
        ARSAL_PRINT (ARSAL_PRINT_INFO, TAG, "-- Starting --");

        // initialize jsMnager
        deviceManager->alManager = NULL;
        deviceManager->netManager = NULL;
        deviceManager->rxThread = NULL;
        deviceManager->txThread = NULL;
        deviceManager->d2cPort = BD_D2C_PORT;
        deviceManager->c2dPort = BD_C2D_PORT; //deviceManager->c2dPort = 0; // Should be read from json

        deviceManager->run = 1;

        deviceManager->dataPCMD.flag = 0;
        deviceManager->dataPCMD.roll = 0;
        deviceManager->dataPCMD.pitch = 0;
        deviceManager->dataPCMD.yaw = 0;
        deviceManager->dataPCMD.gaz = 0;

        deviceManager->flyingState = ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_MAX;
    }

    if (!failed)
    {
        failed = ardiscoveryConnect (deviceManager);
    }

    if (!failed)
    {
        failed = startNetwork (deviceManager);
    }

    if(!failed)
        state = 1;

    if (!failed)
    {
        int cmdSend = sendDate(deviceManager);
        failed = !cmdSend;
    }

    if (!failed)
    {
        int cmdSend = sendAllSettings(deviceManager);
        failed = !cmdSend;
    }

    if (!failed)
    {
        int cmdSend = sendAllStates(deviceManager);
        failed = !cmdSend;
    }

    if (!failed)
    {
        // allocate reader thread array.
        deviceManager->readerThreads = calloc(numOfCommandBufferIds, sizeof(ARSAL_Thread_t));

        if (deviceManager->readerThreads == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Allocation of reader threads failed.");
            failed = 1;
        }
    }

    if (!failed)
    {
        // allocate reader thread data array.
        deviceManager->readerThreadsData = calloc(numOfCommandBufferIds, sizeof(READER_THREAD_DATA_t));

        if (deviceManager->readerThreadsData == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Allocation of reader threads data failed.");
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create and start reader threads.
        int readerThreadIndex = 0;
        for (readerThreadIndex = 0 ; readerThreadIndex < numOfCommandBufferIds ; readerThreadIndex++)
        {
            // initialize reader thread data
            deviceManager->readerThreadsData[readerThreadIndex].deviceManager = deviceManager;
            deviceManager->readerThreadsData[readerThreadIndex].readerBufferId = commandBufferIds[readerThreadIndex];

            if (ARSAL_Thread_Create(&(deviceManager->readerThreads[readerThreadIndex]), readerRun, &(deviceManager->readerThreadsData[readerThreadIndex])) != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of reader thread failed.");
                failed = 1;
            }
        }
    }

    int chosen = 0;
	pthread_t thread;
	mouse_t *mouse = malloc(sizeof(mouse_t));

    while(!chosen)
    {
        switch (controler)
        {
            case 0:
                break;

            /* Create thread joystick */
            case 1:
                chosen = 1;
                break;

            case 2:
                if(pthread_create(&thread, NULL, joystick, deviceManager)){
                perror("pthread_create");
                return EXIT_FAILURE;
                }
                chosen = 1;
                break;

            case 3:
                if(pthread_create(&thread, NULL, joystick2, deviceManager)){
                perror("pthread_create");
                return EXIT_FAILURE;
                }
                chosen = 1;
                break;

            case 4:
                mouse->argc = data3->argc;
                mouse->argv = data3->argv;
                mouse->deviceManager = deviceManager;

                if(pthread_create(&thread,NULL,mouse3D,mouse)){
                perror("pthread_create");
                return EXIT_FAILURE;
                }
                chosen = 1;
                break;
        }
    }
        // Récupération des informations utiles du drone
        if (!failed)
        {
            registerARCommandsCallbacks (deviceManager);
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "On ne peut pas récupérer les informations utiles du drone");
            failed = 1;
        }

	/* Boucle principale */
while(continuer)
{
    if(keyEvent == 1)
    {
        keyEvent = 0;
        deviceManager->dataPCMD.flag = PCMD.flag;
        deviceManager->dataPCMD.gaz = PCMD.gaz;
        deviceManager->dataPCMD.pitch = PCMD.pitch;
        deviceManager->dataPCMD.roll = PCMD.roll;
        deviceManager->dataPCMD.yaw = PCMD.yaw;
        if(PCMD.takeoff == 1)
        {
            sendTakeoff(deviceManager);
            PCMD.takeoff = 0;
        }
        if(PCMD.landing == 1)
        {
            sendLanding(deviceManager);
            PCMD.landing = 0;
        }
    }
    sendPCMD(deviceManager);
    usleep(500);
 }

    if (!failed)
    {
        stopNetwork (deviceManager);
        free(deviceManager);
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- END --");
}
