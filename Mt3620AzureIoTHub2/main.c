#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>
#include <regex.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include "epoll_timerfd_utilities.h"

#include <applibs/gpio.h>
#include <applibs/log.h>
#include <applibs/wificonfig.h>
#include <applibs/uart.h>

#include "mt3620_rdb.h"
#include "rgbled_utility.h"

#include "Grove.h"
#include "Sensors/GroveTempHumiSHT31.h"
#include "Sensors/GroveRelay.h"
#include "Sensors/GroveOledDisplay96x96.h"
#include "Sensors/GroveLEDButton.h"
#include "Sensors/GroveLightSensor.h"
#include "Sensors/GroveAD7992.h"

// This C application for a MT3620 Reference Development Board (Azure Sphere) demonstrates how to
// connect an Azure Sphere device to an Azure IoT Hub, use the grove shield and the communication in UART with other device
//. To use this sample, you must first
// add the Azure IoT Hub Connected Service reference to the project (right-click
// References -> Add Connected Service -> Azure IoT Hub), which populates this project
// 
//
// The sample leverages these functionalities of the Azure IoT SDK C:
// - Device to cloud messages;
// - Cloud to device messages;
// - Azure sphere to Raspberry in UART(azs:pin3--> ras:pin10 and azs:pin4--> ras:pin8)
// - Grove shield extended carte
//
// A description of this application follows:
// - LED 1 blinks constantly and the rate of blink shows the rate of update data to the cloud. (period activity)
// - Pressing button A toggles the rate at updating data to the cloud(button action)
//   between three values.
//
// - LED 2 flashes red when a message is sent 
//   and flashes yellow when a message is received.
//
// - LED 3 indicates whether network connection to the Azure IoT Hub has been
//   established.(LED3--> network statut)
//


// This sample uses the API for the following Azure Sphere application libraries:
// - gpio (digital input for button);
// - log (messages shown in Visual Studio's Device Output window during debugging);
// - wificonfig (configure WiFi settings);
// - azureiot (interaction with Azure IoT services)
// - uart (azure phere <--> raspberry)

#ifndef AZURE_IOT_HUB_CONFIGURED
#error \
    "WARNING: Please add a project reference to the Connected Service first \
(right-click References -> Add Connected Service)."
#endif

#include "azure_iot_utilities.h"

// An array defining the RGB GPIOs for each LED on the device
static const GPIO_Id ledsPins[3][3] = {
    {MT3620_RDB_LED1_RED, MT3620_RDB_LED1_GREEN, MT3620_RDB_LED1_BLUE}, {MT3620_RDB_LED2_RED, MT3620_RDB_LED2_GREEN, MT3620_RDB_LED2_BLUE}, {MT3620_RDB_LED3_RED, MT3620_RDB_LED3_GREEN, MT3620_RDB_LED3_BLUE}};

static size_t blinkIntervalIndex = 0;
static RgbLedUtility_Colors ledBlinkColor = RgbLedUtility_Colors_Blue;

static const struct timespec blinkIntervals[] = {{120, 0}, {3600,0}, {7200, 0}};
static const size_t blinkIntervalsCount = sizeof(blinkIntervals) / sizeof(*blinkIntervals);

// File descriptors - initialized to invalid value
static int epollFd = -1;
static int gpioLedBlinkRateButtonFd = -1;
static int gpioSendMessageButtonFd = -1;
static int gpioButtonsManagementTimerFd = -1;
static int gpioLed1TimerFd = -1;	
static int gpioLed2TimerFd = -1;
static int azureIotDoWorkTimerFd = -1;
static int OLEDTimerFd = -1;
static int uartFd = -1;

static int lampState = 0; 
static int buzzerState = 0;
// UART number of bytes receiveds
static size_t totalBytesReceived = 0;

// LED state
static RgbLed led1 = RGBLED_INIT_VALUE;
static RgbLed led2 = RGBLED_INIT_VALUE;
static RgbLed led3 = RGBLED_INIT_VALUE;
static RgbLed *rgbLeds[] = {&led1, &led2, &led3};
static const size_t rgbLedsCount = sizeof(rgbLeds) / sizeof(*rgbLeds);

// module of grove shield (buzzer and temperture/humidity sensor)
void *relay;
void* sht31;
void *btn;
void* adc;

// Default updating rate of data to cloud
static struct timespec blinkingLedPeriod = {10, 0}; 
static struct timespec updatePeriod = { 1, 0 };
static bool blinkingLedState;

// A null period to not start the timer when it is created with CreateTimerFdAndAddToEpoll.
static const struct timespec nullPeriod = {0, 0};
static const struct timespec defaultBlinkTimeLed2 = {0, 150 * 1000 * 1000};
static void SendMessageToIotHub(char* message);
// Connectivity state
static bool connectedToIoTHub = false;

// Termination state
static volatile sig_atomic_t terminationRequired = false;

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    terminationRequired = true;
}

/// <summary>
///  hepler function to get Timestamp 
/// </summary>
///<retrun name="strTime"> the actual time stamp </retrun>
char strTime[35];
char *GetTimestamp(void)
{
	time_t now;
	time(&now);
	sprintf(strTime, "%d", now);
	strcat(strTime, "000");

	return strTime;
}

/// <summary>
///     GET THE ACTUAL TIME
/// </summary>
/// <return>The logcal time in string </return>
/// 
char strTime1[100];
char* localTimer()
{
	time_t time_T;
	time_T = time(NULL);
	struct tm *tmTime;
	tmTime = localtime(&time_T);
	char* format = "%m-%d %H:%M:%S";

	strftime(strTime1, sizeof(strTime1), format, tmTime);
	return strTime1;
}

/// <summary>
///     Update the information every second in the OLED
/// </summary>
static void OLEDTimerEventHandler(event_data_t *eventData)
{
	if (ConsumeTimerFdEvent(OLEDTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	GroveTempHumiSHT31_Read(sht31);
	float temp = GroveTempHumiSHT31_GetTemperature(sht31);
	float humi = GroveTempHumiSHT31_GetHumidity(sht31);
	
	int state = 0;
	
	float distance = GroveLightSensor_Read(adc);
	distance = GroveAD7992_ConvertToMillisVolt(distance);
	
	char a[50] = "\173\"type\":\"Reading\",\"origin\":\"Sphere\",\"timestamp\":";
	char c[50] = ",\"data\":\173\"type\":\"Presence\",\"value\": ";
	char e[10] = " \175\175";
	char f[10];
	char d[200];

	
	if ((uint16_t)distance >= 1500)
	{
		state = 1;
		strcpy(d, a);
		strcat(d, GetTimestamp());
		strcat(d, c);
		sprintf(f, "%d", state);
		strcat(d, f);
		strcat(d, e);
		SendMessageToIotHub(d);
	}
	else
	{
		state = 0;
		strcpy(d, a);
		strcat(d, GetTimestamp());
		strcat(d, c);
		sprintf(f, "%d", state);
		strcat(d, f);
		strcat(d, e);
		SendMessageToIotHub(d);
		SendMessageToIotHub(d);
	}



	strcpy(d, a);
	strcat(d, GetTimestamp());
	strcat(d, c);
	sprintf(f, "%d", state);
	strcat(d, f);
	strcat(d, e);
	SendMessageToIotHub(d);

	
	setNormalDisplay();
	setTextXY(1, 0);
	putString("temp:");
	putNumber(temp); //Print temp
	putString("deg");
	setTextXY(2, 0);
	putString("humi:");
	putNumber(humi); //Print humi
	putString("%");
	setNormalDisplay();
	setTextXY(3, 0);
	putString(localTimer());
	setTextXY(5,0);
	putString("Lamp:");
	setTextXY(5, 6);
	
	
	if (lampState==1)
	{
		
		putString("On ");
	}
	else
	{
		putString("Off");

	}
	setTextXY(6, 0);
	putString("Buzzer:");
	setTextXY(6, 8);
	if (buzzerState == 1)
	{	
		putString("On ");
	}
	else
	{
		putString("Off");

	}
	
	
	
	
}

/// <summary>
///     Helper function to send a fixed message via the given UART.
/// </summary>
/// <param name="uartFd">The open file descriptor of the UART to write to</param>
/// <param name="dataToSend">The data to send over the UART</param>
static void SendUartMessage(int uartFd, const char *dataToSend)
{
	size_t totalBytesSent = 0;
	size_t totalBytesToSend = strlen(dataToSend);
	int sendIterations = 0;
	while (totalBytesSent < totalBytesToSend) {
		sendIterations++;

		// Send as much of the remaining data as possible
		size_t bytesLeftToSend = totalBytesToSend - totalBytesSent;
		const char *remainingMessageToSend = dataToSend + totalBytesSent;
		ssize_t bytesSent = write(uartFd, remainingMessageToSend, bytesLeftToSend);
		if (bytesSent < 0) {
			Log_Debug("ERROR: Could not write to UART: %s (%d).\n", strerror(errno), errno);
			terminationRequired = true;
			return;
		}

		totalBytesSent += (size_t)bytesSent;
	}

	Log_Debug("Sent %zu bytes over UART in %d calls.\n", totalBytesSent, sendIterations);
}

/// <summary>
///     Handle UART event: if there is incoming data, print it and send it to the cloud
/// </summary>
static void UartEventHandler(event_data_t *eventData)
{
	const size_t receiveBufferSize = 256;
	uint8_t receiveBuffer[receiveBufferSize + 1]; // allow extra byte for string termination
	ssize_t bytesRead;

	// Read UART message
	bytesRead = read(uartFd, receiveBuffer, receiveBufferSize);
	if (bytesRead < 0) {
		Log_Debug("ERROR: Could not read UART: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (bytesRead > 0) {
		// Null terminate the buffer to make it a valid string, and print it
		receiveBuffer[bytesRead] = 0;
		//Log_Debug("UART received %d bytes: '%s'.\n", bytesRead, receiveBuffer);


		// If message receive is lightOn turn on, else turn off
		totalBytesReceived += (size_t)bytesRead;
		Log_Debug(receiveBuffer);

		if (strcmp((char*)receiveBuffer, "lightOn") == 0) {
			GroveLEDButton_LedOn(btn);
			lampState = 1;
			
		}
		else if (strcmp((char*)receiveBuffer, "lightOff") == 0) {
			GroveLEDButton_LedOff(btn);
			lampState = 0;
		}
		else if (strcmp((char*)receiveBuffer, "tempT") == 0) {
			float temp = GroveTempHumiSHT31_GetTemperature(sht31);
			char f[10];
			sprintf(f, "%f", temp);
			SendUartMessage(uartFd,f);
		}


	}
}

/// <summary>
///     Show details of the currently connected WiFi network.(not important)
/// </summary>
static void DebugPrintCurrentlyConnectedWiFiNetwork(void)
{
    WifiConfig_ConnectedNetwork network;
    int result = WifiConfig_GetCurrentNetwork(&network);
    if (result < 0) {
        Log_Debug("INFO: Not currently connected to a WiFi network.\n");
    } else {
        Log_Debug("INFO: Currently connected WiFi network: \n");
        Log_Debug("INFO: SSID \"%.*s\", BSSID %02x:%02x:%02x:%02x:%02x:%02x, Frequency %dMHz.\n",
                  network.ssidLength, network.ssid, network.bssid[0], network.bssid[1],
                  network.bssid[2], network.bssid[3], network.bssid[4], network.bssid[5],
                  network.frequencyMHz);
    }
}

/// <summary>
///     Helper function to blink LED2 once.
/// </summary>
static void BlinkLed2Once(void)
{
    RgbLedUtility_SetLed(&led2, RgbLedUtility_Colors_Red);
    SetTimerFdToSingleExpiry(gpioLed2TimerFd, &defaultBlinkTimeLed2);
}

/// <summary>
///     Helper function to open a file descriptor for the given GPIO as input mode.
/// </summary>
/// <param name="gpioId">The GPIO to open.</param>
/// <param name="outGpioFd">File descriptor of the opened GPIO.</param>
/// <returns>True if successful, false if an error occurred.</return>
static bool OpenGpioFdAsInput(GPIO_Id gpioId, int *outGpioFd)
{
    *outGpioFd = GPIO_OpenAsInput(gpioId);
    if (*outGpioFd < 0) {
        Log_Debug("ERROR: Could not open GPIO '%d': %d (%s).\n", gpioId, errno, strerror(errno));
        return false;
    }

    return true;
}

/// <summary>
///     Toggles the update speed between 3 values and the LED1 blinking speed
/// </summary>
/// <param name="rate">The updating rate</param>
static void SetLedRate(const struct timespec *rate)
{
    if (SetTimerFdToPeriod(gpioLed1TimerFd, rate) != 0) {
        Log_Debug("ERROR: could not set the period of the LED.\n");
        terminationRequired = true;
        return;
    }

    if (connectedToIoTHub) {
        // Report the current state to the Device Twin on the IoT Hub.
        AzureIoT_TwinReportState("LedBlinkRateProperty", blinkIntervalIndex);
    } else {
        Log_Debug("WARNING: Cannot send reported property; not connected to the IoT Hub.\n");
    }
}

/// <summary>
///     Sends a message to the IoT Hub.
/// </summary>
/// <param name = "message"> The message will be send to the cloud</param>
static void SendMessageToIotHub(char* message)
{
    if (connectedToIoTHub) {
        // Send a message
        AzureIoT_SendMessage(message);

        // Set the send/receive LED2 to blink once immediately to indicate the message has been
        // queued.
        BlinkLed2Once();
    } else {
        Log_Debug("WARNING: Cannot send message: not connected to the IoT Hub.\n");
    }
}

/// <summary>
///     MessageReceived callback function, called when a message is received from the Azure IoT Hub.
/// </summary>
/// <param name="payload">The payload of the received message.</param>
static void MessageReceived(const char *payload)
{

	int statuts;
	JSON_Value * json = json_parse_string(payload);
	char *commande = json_object_get_string(json_object_get_object(json_object(json), "Data"), "type");
	 statuts = json_object_get_number(json_object_get_object(json_object(json), "Data"), "value");

	 if (commande==NULL)
	 {
		 return 0; 
	 }
	 else
	 {
		 if (strcmp(commande, "SetLight") == 0)
		 {
			 if (statuts == 1) {
				 GroveLEDButton_LedOn(btn);
				 Log_Debug("light on\n");
				 lampState = 1;
			 }
			 else {
				 GroveLEDButton_LedOff(btn);
				 Log_Debug("light off\n");
				 lampState = 0;
			 }

		 }
		 else if (strcmp(commande, "SetAlarm") == 0)
		 {
			 if (statuts == 1)
			 {
				 GroveRelay_On(relay);
				 buzzerState = 1;
			 }
			 else
			 {
				 GroveRelay_Off(relay);
				 buzzerState = 0;
			 }
		 }


		 else
		 {
			 Log_Debug("error \n");
		 }

		 json_value_free(json);
	 }
    // Set the send/receive LED2 to blink once immediately to indicate a message has been received.
    BlinkLed2Once();
}

/// <summary>
///     Device Twin update callback function, called when an update is received from the Azure IoT
///     Hub.
/// </summary>
/// <param name="desiredProperties">The JSON root object containing the desired Device Twin
/// properties received from the Azure IoT Hub.</param>
static void DeviceTwinUpdate(JSON_Object *desiredProperties)
{
    JSON_Value *blinkRateJson = json_object_get_value(desiredProperties, "LedBlinkRateProperty");

    // If the attribute is missing or its type is not a number.
    if (blinkRateJson == NULL) {
        Log_Debug(
            "INFO: A device twin update was received that did not contain the property "
            "\"LedBlinkRateProperty\".\n");
    } else if (json_value_get_type(blinkRateJson) != JSONNumber) {
        Log_Debug(
            "INFO: Device twin desired property \"LedBlinkRateProperty\" was received with "
            "incorrect type; it must be an integer.\n");
    } else {
        // Get the value of the LedBlinkRateProperty and print it.
        size_t desiredBlinkRate = (size_t)json_value_get_number(blinkRateJson);

        blinkIntervalIndex =
            desiredBlinkRate % blinkIntervalsCount; // Clamp value to [0..blinkIntervalsCount) .

        Log_Debug("INFO: Received desired value %zu for LedBlinkRateProperty, setting it to %zu.\n",
                  desiredBlinkRate, blinkIntervalIndex);

        blinkingLedPeriod = blinkIntervals[blinkIntervalIndex];
        SetLedRate(&blinkIntervals[blinkIntervalIndex]);
    }
}

/// <summary>
///     Allocates and formats a string message on the heap.
/// </summary>
/// <param name="messageFormat">The format of the message</param>
/// <param name="maxLength">The maximum length of the formatted message string</param>
/// <returns>The pointer to the heap allocated memory.</returns>
static void *SetupHeapMessage(const char *messageFormat, size_t maxLength, ...)
{
    va_list args;
    va_start(args, maxLength);
    char *message =
        malloc(maxLength + 1); // Ensure there is space for the null terminator put by vsnprintf.
    if (message != NULL) {
        vsnprintf(message, maxLength, messageFormat, args);
    }
    va_end(args);
    return message;
}

/// <summary>
///     Direct Method callback function, called when a Direct Method call is received from the Azure
///     IoT Hub.
/// </summary>
/// <param name="methodName">The name of the method being called.</param>
/// <param name="payload">The payload of the method.</param>
/// <param name="responsePayload">The response payload content. This must be a heap-allocated
/// string, 'free' will be called on this buffer by the Azure IoT Hub SDK.</param>
/// <param name="responsePayloadSize">The size of the response payload content.</param>
/// <returns>200 HTTP status code if the method name is "LedColorControlMethod" and the color is
/// correctly parsed;
/// 400 HTTP status code is the color has not been recognised in the payload;
/// 404 HTTP status code if the method name is unknown.</returns>
static int DirectMethodCall(const char *methodName, const char *payload, size_t payloadSize,
                            char **responsePayload, size_t *responsePayloadSize)
{
    // Prepare the payload for the response. This is a heap allocated null terminated string.
    // The Azure IoT Hub SDK is responsible of freeing it.
    *responsePayload = NULL;  // Reponse payload content.
    *responsePayloadSize = 0; // Response payload content size.

    int result = 404; // HTTP status code.

    if (strcmp(methodName, "LedColorControlMethod") != 0) {
        result = 404;
        Log_Debug("INFO: Method not found called: '%s'.\n", methodName);

        static const char noMethodFound[] = "\"method not found '%s'\"";
        size_t responseMaxLength = sizeof(noMethodFound) + strlen(methodName);
        *responsePayload = SetupHeapMessage(noMethodFound, responseMaxLength, methodName);
        if (*responsePayload == NULL) {
            Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");
            abort();
        }
        *responsePayloadSize = strlen(*responsePayload);
        return result;
    }

    RgbLedUtility_Colors ledColor = RgbLedUtility_Colors_Unknown;
    // The payload should contains JSON such as: { "color": "red"}
    char *directMethodCallContent = malloc(payloadSize + 1); // +1 to store null char at the end.
    if (directMethodCallContent == NULL) {
        Log_Debug("ERROR: Could not allocate buffer for direct method request payload.\n");
        abort();
    }

    memcpy(directMethodCallContent, payload, payloadSize);
    directMethodCallContent[payloadSize] = 0; // Null terminated string.
    JSON_Value *payloadJson = json_parse_string(directMethodCallContent);
    if (payloadJson == NULL) {
        goto colorNotFound;
    }
    JSON_Object *colorJson = json_value_get_object(payloadJson);
    if (colorJson == NULL) {
        goto colorNotFound;
    }
    const char *colorName = json_object_get_string(colorJson, "color");
    if (colorName == NULL) {
        goto colorNotFound;
    }

    ledColor = RgbLedUtility_GetColorFromString(colorName, strlen(colorName));

    // If color's name has not been identified.
    if (ledColor == RgbLedUtility_Colors_Unknown) {
        goto colorNotFound;
    }

    // Color's name has been identified.
    result = 200;
    const char *colorString = RgbLedUtility_GetStringFromColor(ledColor);
    Log_Debug("INFO: LED color set to: '%s'.\n", colorString);
    // Set the blinking LED color.
    ledBlinkColor = ledColor;

    static const char colorOkResponse[] =
        "{ \"success\" : true, \"message\" : \"led color set to %s\" }";
    size_t responseMaxLength = sizeof(colorOkResponse) + strlen(payload);
    *responsePayload = SetupHeapMessage(colorOkResponse, responseMaxLength, colorString);
    if (*responsePayload == NULL) {
        Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");
        abort();
    }
    *responsePayloadSize = strlen(*responsePayload);

    return result;

colorNotFound:
    result = 400; // Bad request.
    Log_Debug("INFO: Unrecognised direct method payload format.\n");

    static const char noColorResponse[] =
        "{ \"success\" : false, \"message\" : \"request does not contain an identifiable "
        "color\" }";
    responseMaxLength = sizeof(noColorResponse);
    *responsePayload = SetupHeapMessage(noColorResponse, responseMaxLength);
    if (*responsePayload == NULL) {
        Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");
        abort();
    }
    *responsePayloadSize = strlen(*responsePayload);

    return result;
}

/// <summary>
///     IoT Hub connection status callback function.
/// </summary>
/// <param name="connected">'true' when the connection to the IoT Hub is established.</param>
static void IoTHubConnectionStatusChanged(bool connected)
{
    connectedToIoTHub = connected;
}


/// <summary>
///     Handle the updating information
/// </summary>
static void Led1UpdateHandler(event_data_t *eventData)
{
    if (ConsumeTimerFdEvent(gpioLed1TimerFd) != 0) {
        terminationRequired = true;
        return;
    }

    // Set network status with LED3 color.
    RgbLedUtility_Colors color =
        (connectedToIoTHub ? RgbLedUtility_Colors_Green : RgbLedUtility_Colors_Off);
    RgbLedUtility_SetLed(&led3, color);


    // Trigger LED to blink as appropriate.
    blinkingLedState = !blinkingLedState;
    color = (blinkingLedState ? ledBlinkColor : RgbLedUtility_Colors_Off);
    RgbLedUtility_SetLed(&led1, color);

	GroveTempHumiSHT31_Read(sht31);
	float temp = GroveTempHumiSHT31_GetTemperature(sht31);
	float humi = GroveTempHumiSHT31_GetHumidity(sht31);

	char a[50] = "\173\"type\":\"Reading\",\"origin\":\"Sphere\",\"timestamp\":";
	char c[50] = ",\"data\":\173\"type\":\"Temperature\",\"value\": ";
	char c1[50] = ",\"data\":\173\"type\":\"Humidity\",\"value\": ";
	char e[10] = " \175\175";
	char f[10];
	char f1[10];
	char d[200];
	char d1[200];

	strcpy(d, a);
	strcat(d, GetTimestamp());
	strcat(d, c);
	sprintf(f, "%f", temp);
	strcat(d, f);
	strcat(d, e);
	
	SendMessageToIotHub(d);

	strcpy(d1, a);
	strcat(d1, GetTimestamp());
	strcat(d1, c1);
	sprintf(f1, "%f", humi);
	strcat(d1, f1);
	strcat(d1, e);
	sprintf(f1, "%f", humi);
	SendMessageToIotHub(d1);
	Log_Debug("Temperature: %.1fC\n", temp);
	Log_Debug("Humidity: %.1f\%c\n", humi, 0x25);
}



/// <summary>
///     Handle the blinking for LED2.
/// </summary>
static void Led2UpdateHandler(event_data_t *eventData)
{
    if (ConsumeTimerFdEvent(gpioLed2TimerFd) != 0) {
        terminationRequired = true;
        return;
    }

    // Clear the send/receive LED2.
    RgbLedUtility_SetLed(&led2, RgbLedUtility_Colors_Off);
}

/// <summary>
///     Check whether a given button has just been pressed.
/// </summary>
/// <param name="fd">The button file descriptor</param>
/// <param name="oldState">Old state of the button (pressed or released)</param>
/// <returns>true if pressed, false otherwise</returns>
static bool IsButtonPressed(int fd, GPIO_Value_Type *oldState)
{
    bool isButtonPressed = false;
    GPIO_Value_Type newState;
    int result = GPIO_GetValue(fd, &newState);
    if (result != 0) {
        Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
        terminationRequired = true;
    } else {
        // Button is pressed if it is low and different than last known state.
        isButtonPressed = (newState != *oldState) && (newState == GPIO_Value_Low);
        *oldState = newState;
    }

    return isButtonPressed;
}

/// <summary>
///     Handle button timer event: if the button is pressed, change the LED blink rate.
/// </summary>
static void ButtonsHandler(event_data_t *eventData)
{
    if (ConsumeTimerFdEvent(gpioButtonsManagementTimerFd) != 0) {
        terminationRequired = true;
        return;
    }

    // If the button1 is pressed, change the LED blink interval, and update the Twin Device.
    static GPIO_Value_Type blinkButtonState;
    if (IsButtonPressed(gpioLedBlinkRateButtonFd, &blinkButtonState)) {
        blinkIntervalIndex = (blinkIntervalIndex + 1) % blinkIntervalsCount;
        SetLedRate(&blinkIntervals[blinkIntervalIndex]);
    }

    // If the button2 is pressed, send a message to the IoT Hub.
    static GPIO_Value_Type messageButtonState;
    if (IsButtonPressed(gpioSendMessageButtonFd, &messageButtonState)) {
        SendMessageToIotHub("test");
    }
}

/// <summary>
///     Hand over control periodically to the Azure IoT SDK's DoWork.
/// </summary>
static void AzureIotDoWorkHandler(event_data_t *eventData)
{
    if (ConsumeTimerFdEvent(azureIotDoWorkTimerFd) != 0) {
        terminationRequired = true;
        return;
    }

    // Set up the connection to the IoT Hub client.
    // Notes it is safe to call this function even if the client has already been set up, as in
    //   this case it would have no effect
    if (AzureIoT_SetupClient()) {
        // AzureIoT_DoPeriodicTasks() needs to be called frequently in order to keep active
        // the flow of data with the Azure IoT Hub
        AzureIoT_DoPeriodicTasks();
    }
}

// event handler data structures. Only the event handler field needs to be populated.
static event_data_t buttonsEventData = {.eventHandler = &ButtonsHandler};
static event_data_t led1EventData = {.eventHandler = &Led1UpdateHandler};
static event_data_t led2EventData = {.eventHandler = &Led2UpdateHandler};
static event_data_t azureIotEventData = {.eventHandler = &AzureIotDoWorkHandler};
static event_data_t uartEventData = { .eventHandler = &UartEventHandler };
static event_data_t oledEventData = { .eventHandler = &OLEDTimerEventHandler };


/// <summary>
///     Initialize peripherals, termination handler, uart, grove shield and Azure IoT
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
    // Register a SIGTERM handler for termination requests
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

	epollFd = CreateEpollFd();
	if (epollFd < 0) {
		return -1;
	}
	
// Create a UART_Config object, open the UART and set up UART event handler
	/*baudrate = 9600;
	Port3;
	non parity (didn't set);
	non stop bit (didn't set);
	non flow control*/

	UART_Config uartConfig;
	UART_InitConfig(&uartConfig);
	uartConfig.baudRate = 9600;
	uartConfig.flowControl = UART_FlowControl_None;
	uartFd = UART_Open(MT3620_RDB_HEADER3_ISU3_UART , &uartConfig);
	
	//Check if the uart port can be opened or not
	if (uartFd < 0) {
		Log_Debug("ERROR: Could not open UART: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	// Put into the epoll
	if (RegisterEventHandlerToEpoll(epollFd, uartFd, &uartEventData, EPOLLIN) != 0) {
		return -1;
	}
	
	//Configurate the grove shield card
	//OLED display 
	//temp&humi sensor
	//Buzzer
	
	int i2cFd;
	GroveShield_Initialize(&i2cFd, 230400);
	GroveOledDisplay_Init(i2cFd, SH1107G);
	
	clearDisplay();
	setNormalDisplay();
	putString("Initializing..."); 
	usleep(1000000);
	clearDisplay();
	sht31 = GroveTempHumiSHT31_Open(i2cFd);

	adc = GroveLightSensor_Init(i2cFd, 0);
	relay = GroveRelay_Open(4);
	btn = GroveLEDButton_Init(1, 0);
	
    // Open button A
    Log_Debug("INFO: Opening MT3620_RDB_BUTTON_A.\n");
    if (!OpenGpioFdAsInput(MT3620_RDB_BUTTON_A, &gpioLedBlinkRateButtonFd)) {
        return -1;
    }

    // Open button B
    Log_Debug("INFO: Opening MT3620_RDB_BUTTON_B.\n");
    if (!OpenGpioFdAsInput(MT3620_RDB_BUTTON_B, &gpioSendMessageButtonFd)) {
        return -1;
    }

    // Open file descriptors for the RGB LEDs and store them in the rgbLeds array (and in turn in
    // the ledBlink, ledMessageEventSentReceived, ledNetworkStatus variables)
    RgbLedUtility_OpenLeds(rgbLeds, rgbLedsCount, ledsPins);

    // Initialize the Azure IoT SDK
    if (!AzureIoT_Initialize()) {
        Log_Debug("ERROR: Cannot initialize Azure IoT Hub SDK.\n");
        return -1;
    }

    // Set the Azure IoT hub related callbacks only the function send and receive will be used
    AzureIoT_SetMessageReceivedCallback(&MessageReceived);
    AzureIoT_SetDeviceTwinUpdateCallback(&DeviceTwinUpdate);//no use
    AzureIoT_SetDirectMethodCallback(&DirectMethodCall);//no use
    AzureIoT_SetConnectionStatusCallback(&IoTHubConnectionStatusChanged);

    // Display the currently connected WiFi connection.
    DebugPrintCurrentlyConnectedWiFiNetwork();

    // Set up a timer for updating information frequency (periode 3s for the moment)
    gpioLed1TimerFd =
        CreateTimerFdAndAddToEpoll(epollFd, &blinkingLedPeriod, &led1EventData, EPOLLIN);
    if (gpioLed1TimerFd < 0) {
        return -1;
    }

    // Set up a timer for blinking LED2 once.
    gpioLed2TimerFd = CreateTimerFdAndAddToEpoll(epollFd, &nullPeriod, &led2EventData, EPOLLIN);
    if (gpioLed2TimerFd < 0) {
        return -1;
    }

    // Set up a timer for buttons status check
    static struct timespec buttonsPressCheckPeriod = {0, 1000000};
    gpioButtonsManagementTimerFd =
        CreateTimerFdAndAddToEpoll(epollFd, &buttonsPressCheckPeriod, &buttonsEventData, EPOLLIN);
    if (gpioButtonsManagementTimerFd < 0) {
        return -1;
    }

    // Set up a timer for Azure IoT SDK DoWork execution.
    static struct timespec azureIotDoWorkPeriod = {1, 0};
    azureIotDoWorkTimerFd =
        CreateTimerFdAndAddToEpoll(epollFd, &azureIotDoWorkPeriod, &azureIotEventData, EPOLLIN);
    if (azureIotDoWorkTimerFd < 0) {
        return -1;
    }

	// Set up a timer for OLED display every sencond
	OLEDTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &updatePeriod,
		&oledEventData, EPOLLIN);
	if (OLEDTimerFd < 0) {
		return -1;
	}

    return 0;
}

/// <summary>
///     Close peripherals and Azure IoT and UART
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    Log_Debug("INFO: Closing GPIOs and Azure IoT client.\n");

    // Close all file descriptors
    CloseFdAndPrintError(gpioLedBlinkRateButtonFd, "LedBlinkRateButton");
    CloseFdAndPrintError(gpioSendMessageButtonFd, "SendMessageButton");
    CloseFdAndPrintError(gpioButtonsManagementTimerFd, "ButtonsManagementTimer");
    CloseFdAndPrintError(azureIotDoWorkTimerFd, "IotDoWorkTimer");
    CloseFdAndPrintError(gpioLed1TimerFd, "Led1Timer");
	CloseFdAndPrintError(OLEDTimerFd, "OLEDTimerFd");
	CloseFdAndPrintError(gpioLed2TimerFd, "Led2Timer");
    CloseFdAndPrintError(epollFd, "Epoll");
	
    // Close the LEDs and leave then off
    RgbLedUtility_CloseLeds(rgbLeds, rgbLedsCount);

    // Destroy the IoT Hub client
    AzureIoT_DestroyClient();
    AzureIoT_Deinitialize();
}

/// <summary>
///     Main entry point for this application.
/// </summary>
int main(int argc, char *argv[])
{
	
    Log_Debug("INFO: PING23 application starting.\n");

    int initResult = InitPeripheralsAndHandlers();
    if (initResult != 0) {
        terminationRequired = true;
    }

    while (!terminationRequired) {
        if (WaitForEventAndCallHandler(epollFd) != 0) {
            terminationRequired = true;
        }
    }

    ClosePeripheralsAndHandlers();
    Log_Debug("INFO: Application exiting.\n");
    return 0;
}
