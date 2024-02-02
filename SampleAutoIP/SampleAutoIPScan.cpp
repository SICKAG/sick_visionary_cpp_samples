//
// Copyright (c) 2023 SICK AG, Waldkirch
//
// SPDX-License-Identifier: Unlicense

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "VisionaryAutoIPScan.h"

static int runScanDemo(const std::string& hostIp,
                       std::uint8_t       prefixLength,
                       std::uint16_t      broadcastPort,
                       unsigned int       broadcastTimeoutMs)
{
  using namespace visionary;

  VisionaryAutoIPScan ipScan(hostIp, prefixLength);

  // scan for devices
  std::vector<VisionaryAutoIPScan::DeviceInfo> deviceList = ipScan.doScan(broadcastTimeoutMs, broadcastPort);

  std::unordered_set<std::string> deviceMacs{};
  // print device info for every found device
  for (auto it : deviceList)
  {
    std::string mac    = VisionaryAutoIPScan::convertMacToString(it.macAddress);
    auto        status = deviceMacs.insert(mac);
    if (!status.second)
      continue;
    std::cout << "Device name:  " << it.deviceName << '\n'
              << "MAC Address:  " << mac << '\n'
              << "IP Address:   " << it.ipAddress << '\n'
              << "Subnet:       " << it.subNet << '\n'
              << "Control port: " << it.port << std::endl;
  }
  std::cout << '\n' << "Number of found devices: " << deviceMacs.size() << std::endl;

  return 0;
}

int main(int argc, char* argv[])
{
  using namespace visionary;

  constexpr unsigned DEF_BROADCAST_TIMEOUT = 5000u;

  std::string   hostIP;
  std::uint16_t broadcastPort     = VisionaryAutoIPScan::DEFAULT_PORT;
  unsigned int broadcastTimeoutMs = DEF_BROADCAST_TIMEOUT; // The time how long to wait for a response from the devices.

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
      case 'i':
        argstream >> hostIP;
        break;
      case 'p':
        argstream >> broadcastPort;
        break;
      case 't':
        argstream >> broadcastTimeoutMs;
        break;
      default:
        showHelpAndExit = true;
        exitCode        = 1;
        break;
    }
  }

  std::replace(hostIP.begin(), hostIP.end(), '/', ' ');
  std::istringstream ipStream(hostIP);
  std::string        ip;
  std::uint16_t      prefix;
  if (ipStream >> ip >> prefix)
  {
    if (prefix > 32)
      showHelpAndExit = true;
  }
  else
  {
    showHelpAndExit = true;
  }

  if (showHelpAndExit)
  {
    std::cout << argv[0] << " [option]*" << std::endl;
    std::cout << "where option is one of" << std::endl;
    std::cout << "-h          show this help and exit" << std::endl;
    std::cout << "-i<IP>      ip address of the host in a CIDR manner, " << std::endl
              << "            i.e., using ip address and the length of network prefix seperated by /. " << std::endl
              << "            For example, -i192.168.1.100/24" << std::endl
              << "            Note the range of prefix is [0, 32]. " << std::endl;
    std::cout << "-p<port>    broadcast port to use; default is " << visionary::VisionaryAutoIPScan::DEFAULT_PORT
              << std::endl;
    std::cout << "-t<timeout> broadcast timeout in milliseconds; default is " << DEF_BROADCAST_TIMEOUT << std::endl;

    return exitCode;
  }

  return runScanDemo(ip, static_cast<uint8_t>(prefix), broadcastPort, broadcastTimeoutMs);
}
