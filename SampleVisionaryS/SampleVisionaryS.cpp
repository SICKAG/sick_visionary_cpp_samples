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
#include "VisionarySData.h" // Header specific for the Stereo data

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

static int runStreamingDemo(const std::string& ipAddress, unsigned short dataPort, unsigned numberOfFrames)
{
  using namespace visionary;

  ExitCode exitcode;

  // Generate Visionary instance
  auto                pDataHandler = std::make_shared<VisionarySData>();
  VisionaryDataStream dataStream(pDataHandler);
  VisionaryControl    visionaryControl;

  //-----------------------------------------------
  // Connect to devices control channel
  if (!visionaryControl.open(VisionaryControl::ProtocolType::COLA_B, ipAddress, 5000 /*ms*/))
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
  // Set framePeriod parameter to 150000
  {
    std::printf("Setting framePeriodTime to 150000\n");
    CoLaCommand setFramePeriodCommand =
      CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "framePeriodTime").parameterUDInt(150000).build();
    CoLaCommand setFramePeriodResponse = visionaryControl.sendCommand(setFramePeriodCommand);

    if (setFramePeriodResponse.getError() != CoLaError::OK)
    {
      exitcode(5);
      std::printf("Failed to write the frame period time\n");
    }
    else
    {
      std::printf("Successfully set framePeriodTime to 150000\n");
    }
  }

  //-----------------------------------------------
  // Read framePeriod parameter
  {
    CoLaCommand getFramePeriodCommand = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "framePeriodTime").build();
    CoLaCommand framePeriodResponse   = visionaryControl.sendCommand(getFramePeriodCommand);

    if (framePeriodResponse.getError() != CoLaError::OK)
    {
      exitcode(5);
      std::printf("Failed to read the frame period time\n");
    }
    else
    {
      std::uint32_t framePeriodTime = CoLaParameterReader(framePeriodResponse).readUDInt();
      std::printf("Read framePeriodTime = %d\n", framePeriodTime);
    }
  }
  //-----------------------------------------------
  // Auto Exposure functions

  // This section demonstrates how to use the auto exposure functions by invoking the method
  // 'TriggerAutoExposureParameterized'. It's also shown how the region of interest (ROI) can be set. The sample is
  // based on the AcquisitionModeStereo = NORMAL. */
  {
    std::uint8_t acquisitionModeStereo = 0;
    CoLaCommand  setAcquisitionModeStereoCommand =
      CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "acquisitionModeStereo")
        .parameterUSInt(acquisitionModeStereo)
        .build();
    CoLaCommand setAcquisitionModeStereoResponse = visionaryControl.sendCommand(setAcquisitionModeStereoCommand);

    // Set region of interest (ROI)
    std::uint32_t left   = 160;
    std::uint32_t right  = 480;
    std::uint32_t top    = 128;
    std::uint32_t bottom = 384;

    // Set ROI for Auto Exposure 3D
    CoLaCommand setAutoExposureROICommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "autoExposureROI")
                                              .parameterUDInt(left)
                                              .parameterUDInt(right)
                                              .parameterUDInt(top)
                                              .parameterUDInt(bottom)
                                              .build();
    CoLaCommand setAutoExposureROIResponse = visionaryControl.sendCommand(setAutoExposureROICommand);

    // Set ROI for Auto Exposure RGB
    CoLaCommand setAutoExposureColorROICommand =
      CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "autoExposureColorROI")
        .parameterUDInt(left)
        .parameterUDInt(right)
        .parameterUDInt(top)
        .parameterUDInt(bottom)
        .build();
    CoLaCommand setAutoExposureColorROIResponse = visionaryControl.sendCommand(setAutoExposureColorROICommand);

    // Set ROI for Auto White Balance
    // NOTE: The user is responisble to make sure that the region he sets the ROI to, is actually white.
    CoLaCommand setAutoWhiteBalanceROICommand =
      CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "autoWhiteBalanceROI")
        .parameterUDInt(left)
        .parameterUDInt(right)
        .parameterUDInt(top)
        .parameterUDInt(bottom)
        .build();
    CoLaCommand setAutoWhiteBalanceROIResponse = visionaryControl.sendCommand(setAutoWhiteBalanceROICommand);

    // Read out actual integration time values (before auto exposure was triggered)
    // ATTENTION: This sample is based on the NORMAL acquisition mode; other modes may refer to other integration time
    // variables
    CoLaCommand getIntegrationTimeUsCommand =
      CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "integrationTimeUs").build();
    CoLaCommand   getIntegrationTimeUsResponse = visionaryControl.sendCommand(getIntegrationTimeUsCommand);
    std::uint32_t integrationTimeUs            = CoLaParameterReader(getIntegrationTimeUsResponse).readUDInt();
    std::printf("Read integrationTimeUs = %d\n", integrationTimeUs);

    CoLaCommand getIntegrationTimeUsColorCommand =
      CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "integrationTimeUsColor").build();
    CoLaCommand   getIntegrationTimeUsColorResponse = visionaryControl.sendCommand(getIntegrationTimeUsColorCommand);
    std::uint32_t integrationTimeUsColor = CoLaParameterReader(getIntegrationTimeUsColorResponse).readUDInt();
    std::printf("Read integrationTimeUsColor = %d\n", integrationTimeUsColor);

    /* Info: For White Balance exists no SOPAS variable; the changes are done internally in the device and applied to
       the image. If you open SOPAS and you are running this sample in parallel you can see how the image changes. */

    // Invoke auto exposure method
    if (visionaryControl.login(IAuthentication::UserLevel::SERVICE, "CUST_SERV"))
    {
      for (uint8_t autoType = 0; autoType < 3;
           autoType++) // 0 = Auto Exposure 3D, 1 = Auto Exposure RGB, 2 = Auto White Balance
      {
        std::printf("Invoke method 'TriggerAutoExposureParameterized' (Param: %d) ...\n", autoType);

        CoLaCommand invokeAutoExposureCommand =
          CoLaParameterWriter(CoLaCommandType::METHOD_INVOCATION, "TriggerAutoExposureParameterized")
            .parameterUInt(1)
            .parameterUSInt(autoType)
            .build();
        CoLaCommand autoExposureResponse = visionaryControl.sendCommand(invokeAutoExposureCommand);

        if (autoExposureResponse.getError() != CoLaError::OK)
        {
          std::printf("ERROR: Invoking 'TriggerAutoExposureParameterized' fails! (autoExposureResponse: %d)\n",
                      CoLaParameterReader(autoExposureResponse).readBool());
        }

        // Wait until auto exposure method is finished
        bool      autoExpParamRunning = true;
        long long startTime           = std::chrono::system_clock::now().time_since_epoch().count();
        long long timeNow             = startTime;
        while (autoExpParamRunning)
        {
          CoLaCommand getAutoExpParamRunningCommand =
            CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "autoExposureParameterizedRunning").build();
          CoLaCommand autoExpParamRunningResponse = visionaryControl.sendCommand(getAutoExpParamRunningCommand);
          autoExpParamRunning                     = CoLaParameterReader(autoExpParamRunningResponse).readBool();

          timeNow = std::chrono::system_clock::now().time_since_epoch().count();
          if ((timeNow - startTime)
              <= 10000000000) // 10 sec = 10 000 000 000 ns (time after auto exposure method should be finished)
          {
            std::this_thread::sleep_for(std::chrono::seconds(1));
          }
          else
          {
            std::printf("TIMEOUT: auto exposure function (Param: %d) needs longer than expected!\n", autoType);
          }
        }
      }
    }

    // Read out new integration time values (after auto exposure was triggered)
    getIntegrationTimeUsCommand  = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "integrationTimeUs").build();
    getIntegrationTimeUsResponse = visionaryControl.sendCommand(getIntegrationTimeUsCommand);
    integrationTimeUs            = CoLaParameterReader(getIntegrationTimeUsResponse).readUDInt();
    std::printf("Read integrationTimeUs = %d\n", integrationTimeUs);

    getIntegrationTimeUsColorCommand =
      CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "integrationTimeUsColor").build();
    getIntegrationTimeUsColorResponse = visionaryControl.sendCommand(getIntegrationTimeUsColorCommand);
    integrationTimeUsColor            = CoLaParameterReader(getIntegrationTimeUsColorResponse).readUDInt();
    std::printf("Read integrationTimeUsColor = %d\n", integrationTimeUsColor);
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
    const char plyFilePath[] = "VisionaryS.ply";
    std::printf("Writing frame to %s\n", plyFilePath);
    PointCloudPlyWriter::WriteFormatPLY(plyFilePath, pointCloud, pDataHandler->getRGBAMap(), true);
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
    }
  }

  //-----------------------------------------------
  // Stop acquisition
  visionaryControl.stopAcquisition();

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

  bool showHelpAndExit = false;

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

    return exitCode;
  }

  exitCode = runStreamingDemo(deviceIpAddr, deviceBlobCtrlPort, cnt);

  std::cout << "exit code " << exitCode << std::endl;

  return exitCode;
}
