/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/

#pragma once

#include <ros/transport_publisher_link.h>

namespace ros
{
class ROSCPP_DECL ShmTransportPublisherLink : public TransportPublisherLink
{
public:
  using TransportPublisherLink::TransportPublisherLink;

  bool isShm() const override
  {
      return true;
  }
};
typedef boost::shared_ptr<ShmTransportPublisherLink> ShmTransportPublisherLinkPtr;

} // namespace ros
