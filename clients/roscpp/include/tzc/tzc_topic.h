#ifndef __TZC_TOPIC_HPP__
#define __TZC_TOPIC_HPP__

#include "ros/ros.h"
#include "tzc_publisher.h"
#include "tzc_subscriber.h"

namespace tzc_transport
{

class Topic
{
public:
  Topic(const ros::NodeHandle & parent) {
    nh_ = parent;
  }

  ~Topic() {
  }

  template < class M >
  Publisher< M > advertise(const std::string & topic, uint32_t queue_size, uint32_t mem_size) {
    ros::Publisher pub = nh_.advertise< M >(topic, queue_size);
    return Publisher< M >(pub, topic, mem_size);
  }

  template < class M >
  Subscriber< M > subscribe(const std::string & topic, uint32_t queue_size, void (*fp)(const boost::shared_ptr< const M > &),
                            const ros::TransportHints& hints = ros::TransportHints().tcpNoDelay()) {
    SubscriberCallbackHelper< M > * phlp = new SubscriberCallbackHelper< M >(topic, fp);
    ros::Subscriber sub = nh_.subscribe(topic, queue_size, &SubscriberCallbackHelper< M >::callback, phlp, hints);
    return Subscriber< M >(sub, phlp);
  }

private:
  ros::NodeHandle nh_;

};

} // namespace tzc_transport

#endif // __TZC_TOPIC_HPP__

