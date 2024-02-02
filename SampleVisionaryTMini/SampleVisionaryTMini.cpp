//
// Copyright (c) 2023 SICK AG, Waldkirch
//
// SPDX-License-Identifier: Unlicense

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>

#include <chrono>
#include <thread>

#include "CoLaParameterReader.h"
#include "CoLaParameterWriter.h"
#include "PointCloudPlyWriter.h"
#include "PointXYZ.h"
#include "VisionaryControl.h"
#include "VisionaryDataStream.h"
#include "VisionaryTMiniData.h" // Header specific for the Time of Flight data

// helper class, stores an exit code
// with the policy: a smaller code (>0) has priority over a larger exitcode
// (we use a reverse severity for exit codes: 1 is most important)
class ExitCode
{
public:
  ExitCode() : m_code(0)
  {
  }

  // feed in a new exitcode
  void operator()(int c)
  {
    if ((c != 0) && ((0 == m_code) || (c < m_code)))
      m_code = c;
  }

  // check whether everything is ok (no error code set)
  bool ok() const
  {
    return (m_code == 0);
  }

  // get the final exit code
  int get() const
  {
    return m_code;
  }

private:
  int m_code;
};

static int runStreamingDemo(const std::string& ipAddress,
                            unsigned short     dataPort,
                            unsigned           numberOfFrames,
                            bool               executeExtTrigger)
{
  using namespace visionary;

  ExitCode exitcode;

  // Generate Visionary instance
  auto                pDataHandler = std::make_shared<VisionaryTMiniData>();
  VisionaryDataStream dataStream(pDataHandler);
  VisionaryControl    visionaryControl;

  //-----------------------------------------------
  // Connect to devices control channel
  if (!visionaryControl.open(VisionaryControl::ProtocolType::COLA_2, ipAddress, 5000 /*ms*/))
  {
    std::printf("Failed to open control connection to device.\n");
    exitcode(1);
    return exitcode.get();
  }

  //-----------------------------------------------
  // Stop image acquisition (works always, also when already stopped)
  // Further you should always stop the device before reconfiguring it
  visionaryControl.stopAcquisition();

  //-----------------------------------------------
  // read Device Ident
  std::printf("DeviceIdent: '%s'\n", visionaryControl.getDeviceIdent().c_str());

  //-----------------------------------------------
  // Login as authorized client
  if (!visionaryControl.login(IAuthentication::UserLevel::AUTHORIZED_CLIENT, "CLIENT"))
  {
    std::printf("Failed to log into the device.\n");
    exitcode(2);
    return exitcode.get();
  }

  //-----------------------------------------------
  // An example of reading an writing device parameters is shown here.
  // Use the "SOPAS Communication Interface Description" PDF to determine data types for other variables
  //-----------------------------------------------
  // Set enDepthMask parameter to false
  {
    std::printf("Setting enDepthMask to false\n");
    CoLaCommand setEnDepthMaskCommand =
      CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "enDepthMask").parameterBool(false).build();
    CoLaCommand setEnDepthMaskResponse = visionaryControl.sendCommand(setEnDepthMaskCommand);

    if (setEnDepthMaskResponse.getError() != CoLaError::OK)
    {
      exitcode(5);
      std::printf("Failed to write enDepthMask\n");
    }
    else
    {
      std::printf("Successfully set enDepthMask to false\n");
    }
  }

  //-----------------------------------------------
  // Read humidity parameter
  {
    CoLaCommand getHumidity      = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "humidity").build();
    CoLaCommand humidityResponse = visionaryControl.sendCommand(getHumidity);
    if (humidityResponse.getError() != CoLaError::OK)
    {
      exitcode(5);
      std::printf("Failed to read humidity\n");
    }
    else
    {
      const double humidity = CoLaParameterReader(humidityResponse).readLReal();
      std::printf("Read humidity = %f\n", humidity);
    }
  }

  //-----------------------------------------------
  // Read info messages variable
  {
    CoLaCommand getMessagesCommand = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "MSinfo").build();
    CoLaCommand messagesResponse   = visionaryControl.sendCommand(getMessagesCommand);

    // Read message array, length of array is always 25 items (see MSinfo in PDF).
    CoLaParameterReader reader(messagesResponse);
    for (int i = 0; i < 25; i++) // Read 25 items
    {
      uint32_t errorId    = reader.readUDInt();
      uint32_t errorState = reader.readUDInt();

      // Read ErrTimeType struct members for FirstTime
      uint16_t firstTime_PwrOnCount = reader.readUInt();
      uint32_t firstTime_OpSecs     = reader.readUDInt();
      uint32_t firstTime_TimeOccur  = reader.readUDInt();

      // Read ErrTimeType struct members for LastTime
      uint16_t lastTime_PwrOnCount = reader.readUInt();
      uint32_t lastTime_OpSecs     = reader.readUDInt();
      uint32_t lastTime_TimeOccur  = reader.readUDInt();

      uint16_t    numberOccurrences = reader.readUInt();
      uint16_t    errReserved       = reader.readUInt();
      std::string extInfo           = reader.readFlexString();

      // Write all non-empty info messages to the console
      if (errorId != 0)
      {
        std::printf("Info message [0x%032x], extInfo: %s, number of occurrences: %u\n",
                    errorId,
                    extInfo.c_str(),
                    numberOccurrences);
      }
    }
  }

  //-----------------------------------------------
  {
    CoLaCommand setEnDepthMaskCommand =
      CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "enDepthMask").parameterBool(true).build();
    CoLaCommand setEnDepthMaskResponse = visionaryControl.sendCommand(setEnDepthMaskCommand);
    if (setEnDepthMaskResponse.getError() != CoLaError::OK)
    {
      std::printf("Failed to set enDepthMask to true\n");
      exitcode(5);
    }
  }

  //-----------------------------------------------
  // Logout from device after reading variables.
  if (!visionaryControl.logout())
  {
    std::printf("Failed to logout\n");
    exitcode(2);
  }

  //-----------------------------------------------
  // Depending on the PC we might be too fast for the device configuration
  // Just wait a short time. This should only be necessary after stop
  // (to make sure stop really propagated and you don't get a pending frame)
  // or after a configure to make sure configuration has finished
  std::this_thread::sleep_for(
    std::chrono::milliseconds(100)); // This short deelay is necessary to not have any old frames in the pipeline.

  //-----------------------------------------------
  // Connect to devices data stream
  // This is done after stopping acquisition to ensure that there are no old frames are buffered
  if (!dataStream.open(ipAddress, dataPort))
  {
    std::printf("Failed to open data stream connection to device.\n");
    exitcode(10);
    return exitcode.get();
  }

  //-----------------------------------------------
  // Capture a single frame
  visionaryControl.stepAcquisition();
  if (!dataStream.getNextFrame())
  {
    std::printf("Frame timeout after single step.\n");
    exitcode(11);
  }
  else
  {
    std::printf("Frame received through step called, frame #%" PRIu32 ", timestamp: %" PRIu64 "\n",
                pDataHandler->getFrameNum(),
                pDataHandler->getTimestampMS());

    //-----------------------------------------------
    // Convert data to a point cloud
    std::vector<PointXYZ> pointCloud;
    pDataHandler->generatePointCloud(pointCloud);
    pDataHandler->transformPointCloud(pointCloud);

    //-----------------------------------------------
    // Write point cloud to PLY
    const char plyFilePath[] = "VisionaryT_Mini.ply";
    std::printf("Writing frame to %s\n", plyFilePath);
    PointCloudPlyWriter::WriteFormatPLY(plyFilePath, pointCloud, pDataHandler->getIntensityMap(), true);
    std::printf("Finished writing frame to %s\n", plyFilePath);
  }

  //-----------------------------------------------
  // Start image acquisiton and continously receive frames
  visionaryControl.startAcquisition();
  for (unsigned i = 0u; i < numberOfFrames; i++)
  {
    if (!dataStream.getNextFrame())
    {
      std::printf("Frame timeout in continuous mode after %u frames\n", i);
      exitcode(12);
      // but we continue to capture
    }
    else
    {
      std::printf("Frame received in continuous mode, frame #%" PRIu32 ", timestamp: %" PRIu64 "\n",
                  pDataHandler->getFrameNum(),
                  pDataHandler->getTimestampMS());
      std::vector<uint16_t> intensityMap = pDataHandler->getIntensityMap();
      std::vector<uint16_t> distanceMap  = pDataHandler->getDistanceMap();
      std::vector<uint16_t> stateMap     = pDataHandler->getStateMap();
    }
  }

  //-----------------------------------------------
  // This part of the sample code is skipped by default because not every user has a working IO trigger hardware
  // available. If you want to execute it set variable "executeExtTrigger" in main function to "true".
  if (executeExtTrigger)
  {
    // Capture single frames with external trigger
    // NOTE: This part of the sample only works if you have a working rising egde signal on IO1 which triggers an image!
    std::printf("\n=== Starting external trigger example:\n");
    // Login as authorized client
    if (!visionaryControl.login(IAuthentication::UserLevel::AUTHORIZED_CLIENT, "CLIENT"))
    {
      std::printf("Failed to log into device\n");
      exitcode(2);
    }
    else
    {
      // Set frontendMode to STOP (= 1)
      std::printf("Setting frontendMode to STOP (= 1)\n");
      CoLaCommand setFrontendModeCommand =
        CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "frontendMode").parameterUSInt(1).build();
      CoLaCommand setFrontendModeResponse = visionaryControl.sendCommand(setFrontendModeCommand);
      if (setFrontendModeResponse.getError() != CoLaError::OK)
      {
        std::printf("Failed to set frontendMode to STOP (= 1)\n");
        exitcode(6);
      }

      // Set INOUT1_Function to Trigger (= 7)
      std::printf("Setting DIO1Fnc to Trigger (= 7)\n");
      CoLaCommand setDIO1FncCommand =
        CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "DIO1Fnc").parameterUSInt(7).build();
      CoLaCommand setDIO1FncResponse = visionaryControl.sendCommand(setDIO1FncCommand);
      if (setDIO1FncResponse.getError() != CoLaError::OK)
      {
        std::printf("Failed to set DIO1Fnc to Trigger (= 7)\n");
        exitcode(5);
      }

      // Set INOUT2_Function to TriggerBusy (= 23)
      std::printf("Setting DIO2Fnc to TriggerBusy (= 23)\n");
      CoLaCommand setDIO2FncCommand =
        CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "DIO2Fnc").parameterUSInt(23).build();
      CoLaCommand setDIO2FncResponse = visionaryControl.sendCommand(setDIO2FncCommand);
      if (setDIO2FncResponse.getError() != CoLaError::OK)
      {
        std::printf("Failed to set DIO2Fnc to TriggerBusy (= 23)\n");
        exitcode(5);
      }
    }

    // Re-Connect to device data stream (make sure there are no old images in the pipeline)
    dataStream.close();
    std::this_thread::sleep_for(
      std::chrono::seconds(1)); // This short delay is necessary to not have any old frames in the pipeline.
    if (!dataStream.open(ipAddress, dataPort))
    {
      std::printf("Failed to open data stream connection to device.\n");
      exitcode(10);
      return exitcode.get();
    }

    std::printf("Please enable trigger on IO1 to receive an image:\n");
    long long startTime = std::chrono::system_clock::now().time_since_epoch().count();
    long long timeNow   = startTime;

    // Limited time loop for receiving hardware trigger signals on IO1 pin
    bool      frameReceived  = false;
    long long triggerTimeOut = 100000000; // 10 sec = 100 000 000
    while ((timeNow - startTime) <= triggerTimeOut)
    {
      // Read variable IOValue
      CoLaCommand         getIOValue      = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "IOValue").build();
      CoLaCommand         IOValueResponse = visionaryControl.sendCommand(getIOValue);
      CoLaParameterReader IOValues(IOValueResponse);

      // The parameter reader works like a FILE, so we need to read one value after the next. First IO1, then IO2 etc
      const std::int8_t IOValue1 = IOValues.readSInt();
      const std::int8_t IOValue2 = IOValues.readSInt(); // We need the IOValue of IO2 from the V3SIOsState struct
      std::printf("Read TriggerBusy = %d\n", IOValue2);

      // Receive the next frame
      if (IOValue2 == 0)
      {
        if (dataStream.getNextFrame())
        {
          std::printf("Frame received in external trigger mode, frame #%" PRIu32 "\n", pDataHandler->getFrameNum());
          frameReceived = true;
        }
        timeNow = std::chrono::system_clock::now().time_since_epoch().count();
      }
    }

    if (!frameReceived)
    {
      std::printf("TIMEOUT: No trigger signal received on IO1 within %.2f seconds!\n",
                  static_cast<float>(triggerTimeOut) / 10000000.f);
      exitcode(13);
    }
  }
  //-----------------------------------------------

  visionaryControl.close();
  dataStream.close();

  return exitcode.get();
}

int main(int argc, char* argv[])
{
  // Insert IP and the API port of your camera, aswell as the number of images you want to receive via cmd/terminal
  /// Default values:
  /// IP:        "192.168.1.10"
  /// API-port:  2114

  std::string    deviceIpAddr("192.168.1.10");
  unsigned short deviceBlobCtrlPort = 2114u;
  unsigned       cnt                = 100u;

  bool showHelpAndExit   = false;
  bool executeExtTrigger = false;

  int exitCode = 0;

  for (int i = 1; i < argc; ++i)
  {
    std::istringstream argstream(argv[i]);

    if (argstream.get() != '-')
    {
      showHelpAndExit = true;
      exitCode        = 1;
      break;
    }
    switch (argstream.get())
    {
      case 'h':
        showHelpAndExit = true;
        break;
      case 'c':
        argstream >> deviceBlobCtrlPort;
        break;
      case 'i':
        argstream >> deviceIpAddr;
        break;
      case 'n':
        argstream >> cnt;
        break;
      case 't':
        executeExtTrigger = true;
        break;
      default:
        showHelpAndExit = true;
        exitCode        = 1;
        break;
    }
  }

  if (showHelpAndExit)
  {
    std::cout << argv[0] << " [option]*" << std::endl;
    std::cout << "where option is one of" << std::endl;
    std::cout << "-h          show this help and exit" << std::endl;
    std::cout << "-i<IP>      connect to the device with IP address <IP>; default is 192.168.1.10" << std::endl;
    std::cout << "-c<port>    assume the BLOB control port of the device was configured to <port>; default is 2114"
              << std::endl;
    std::cout << "-n<cnt>     acquire <cnt> frames and stop; default is 100" << std::endl;
    std::cout << "-t          execute external trigger example part" << std::endl;

    return exitCode;
  }

  exitCode = runStreamingDemo(deviceIpAddr, deviceBlobCtrlPort, cnt, executeExtTrigger);

  std::cout << "exit code " << exitCode << std::endl;

  return exitCode;
}
