/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
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
