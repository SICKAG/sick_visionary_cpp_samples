//
// Copyright (c) 2023 SICK AG, Waldkirch
//
// SPDX-License-Identifier: Unlicense

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "VisionaryAutoIPScan.h"

static int runAssignDemo(const std::string&                           destinationMac,
                         visionary::VisionaryAutoIPScan::ProtocolType colaVer,
                         const std::string&                           ipAddr,
                         std::uint8_t                                 prefixLength,
                         bool                                         dhcp,
                         unsigned int                                 timeout,
                         const std::string&                           ipGateway)
{
  using namespace visionary;

  // assume target ip is in host network
  VisionaryAutoIPScan ipScan(ipAddr, prefixLength);

  // Assign IP address
  bool successful = ipScan.assign(
    VisionaryAutoIPScan::convertMacToStruct(destinationMac), colaVer, ipAddr, prefixLength, ipGateway, dhcp, timeout);

  if (successful)
  {
    std::cout << "Successfully assigned ip address" << std::endl;
  }
  else
  {
    std::cout << "Ip address could not be successfully assigned" << std::endl;
  }
  return 0;
}

int main(int argc, char* argv[])
{
  using namespace visionary;

  constexpr unsigned                          DEF_BROADCAST_TIMEOUT = 5000u;
  constexpr VisionaryAutoIPScan::ProtocolType DEF_PROTOCOL_TYPE     = visionary::VisionaryAutoIPScan::COLA_2;

  std::string                       destinationMac;
  std::string                       ipAddr;
  int                               tmpCola;
  bool                              dhcp        = false;
  VisionaryAutoIPScan::ProtocolType colaVersion = DEF_PROTOCOL_TYPE;
  std::string                       ipGateway   = VisionaryAutoIPScan::DEFAULT_GATEWAY;
  unsigned int                      timeoutMs   = DEF_BROADCAST_TIMEOUT;

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
      case 'o':
        argstream >> destinationMac;
        break;
      case 'c':
        argstream >> tmpCola;
        if (tmpCola == 1)
        {
          colaVersion = visionary::VisionaryAutoIPScan::COLA_B;
        }
        if (tmpCola == 2)
        {
          colaVersion = visionary::VisionaryAutoIPScan::COLA_2;
        }
        break;
      case 'i':
        argstream >> ipAddr;
        break;
      case 'd':
        dhcp = true;
        break;
      case 't':
        argstream >> timeoutMs;
        break;
      case 'g':
        argstream >> ipGateway;
        break;
      default:
        showHelpAndExit = true;
        exitCode        = 1;
        break;
    }
  }

  std::replace(ipAddr.begin(), ipAddr.end(), '/', ' ');
  std::istringstream ipStream(ipAddr);
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
    std::cout << "-h            show this help and exit" << std::endl;
    std::cout << "-o<MAC>       mac address of the device to assign" << std::endl;
    std::cout << "-c<version>   cola version (ColaB: 1, Cola2: 2)" << std::endl;
    std::cout << "-i<IP>        ip address of the host in a CIDR manner, " << std::endl
              << "              i.e., using ip address and the length of network prefix seperated by /. " << std::endl
              << "              For example, -i192.168.1.10/24" << std::endl
              << "              Note the range of prefix is [0, 32]. " << std::endl;
    std::cout << "-d            enable dhcp" << std::endl;
    std::cout << "-t<timeout>   broadcast timeout in milliseconds; default is " << DEF_BROADCAST_TIMEOUT << std::endl;
    std::cout << "-g<IP>        gateway of the device" << std::endl;

    return exitCode;
  }

  return runAssignDemo(destinationMac, colaVersion, ip, static_cast<uint8_t>(prefix), dhcp, timeoutMs, ipGateway);
}
