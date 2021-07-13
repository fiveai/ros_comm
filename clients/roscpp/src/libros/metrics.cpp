// Copyright Five AI 2018. All rights reserved.

#include <chrono>
#include <regex>

#include <ros/param.h>
#include <log4cxx/logger.h>
#include <log4cxx/asyncappender.h>
#include <log4cxx/fileappender.h>
#include <log4cxx/patternlayout.h>
#include <boost/filesystem.hpp>

#include "ros/file_log.h"
#include "metrics.hpp"

namespace
{
class metric_to_string_visitor
    : public boost::static_visitor<std::string>
{
public:

    std::string operator()(bool b) const
    {
        return b ? "t" : "f";
    }

    std::string operator()(const std::string& s) const
    {
        return s;
    }

    template<typename Floating, typename std::enable_if<std::is_floating_point<Floating>::value>::type * = nullptr>
    std::string operator()(const Floating d) const
    {
        return std::to_string(d);
    }

    template<typename Integral, typename std::enable_if<
             std::is_integral<Integral>::value && !std::is_same<bool, Integral>::value>::type * = nullptr>
    std::string operator()(const Integral i) const
    {
        return std::to_string(i) + 'i';
    }
};
}

namespace fiveai
{
std::string format_metrics(const std::string& measurement,
                           std::initializer_list<std::pair<std::string, std::string>> tags,
                           std::initializer_list<std::pair<std::string, MetricValue>> fields)
{
    std::string formatted_metrics{measurement};

    for (const auto& tag : tags)
    {
        formatted_metrics += ',';
        formatted_metrics += tag.first;
        formatted_metrics += '=';
        formatted_metrics += tag.second;
    }

    auto infix = ' ';
    for (const auto& field : fields)
    {
        formatted_metrics += infix;
        formatted_metrics += field.first;
        formatted_metrics += '=';
        formatted_metrics += boost::apply_visitor(metric_to_string_visitor(), field.second );
        infix = ',';
    }

    formatted_metrics += ' ';
    formatted_metrics += std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count());

    return formatted_metrics;
}
void log_metrics(const std::string& measurement,
                 const std::string& topic,
                 std::initializer_list<std::pair<std::string, std::string>> tags,
                 std::initializer_list<std::pair<std::string, MetricValue>> fields)
{
    static const std::regex enabled_metrics{[](){
        // Nothing by default
        std::string enabled_metrics{"^$"};
        ros::param::get("/enabled_metrics", enabled_metrics);
        return enabled_metrics;
    }()};

    if (std::regex_match(topic, enabled_metrics))
    {
        static auto logger = log4cxx::Logger::getLogger("ros_comm.msg_metrics");
        logger->info(format_metrics(measurement, tags, fields));
    }
}

void setup_metrics_logger()
{
	using namespace boost::filesystem;
    const auto metrics_filename = path(ros::file_log::getLogFile()).replace_extension("metrics.log").native();

    // this is a bit wasteful to use the pattern layout but log4cxx doesn't have a plain new line layout builtin
    // @todo implement a custom layout that doesn't have to parse a pattern
    log4cxx::LayoutPtr layout{new log4cxx::PatternLayout("\%m\%n")};
    log4cxx::FileAppenderPtr file_appender{new log4cxx::FileAppender(layout, metrics_filename)};

    log4cxx::AsyncAppenderPtr async_appender{new log4cxx::AsyncAppender};
    async_appender->addAppender(file_appender);

    log4cxx::Logger::getLogger("ros_comm.msg_metrics")->setAdditivity(false); // no-inherit from ROS globals
    log4cxx::Logger::getLogger("ros_comm.msg_metrics")->addAppender(async_appender);
}
}