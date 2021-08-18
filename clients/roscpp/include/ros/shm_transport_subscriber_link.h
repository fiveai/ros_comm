/***************************************************************************************************
 * Copyright Five AI Limited 2021
 * All rights reserved

 * This file is released under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. Please refer to the README.md file supplied with this repository.
 ***************************************************************************************************/

#pragma once

#include <ros/transport_subscriber_link.h>

namespace ros
{

class ROSCPP_DECL ShmTransportSubscriberLink : public TransportSubscriberLink
{
public:
  ShmTransportSubscriberLink() = default;

  void getPublishTypes(bool& ser, bool& nocopy, const std::type_info& /*ti*/) override
  {
      ser = false;
      nocopy = true;
  }

};
typedef boost::shared_ptr<ShmTransportSubscriberLink> ShmTransportSubscriberLinkPtr;

} // namespace ros
