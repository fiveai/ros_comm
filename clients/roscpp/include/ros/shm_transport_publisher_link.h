/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
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
