// Copyright Five AI 2018. All rights reserved.

#include <string>
#include <type_traits>
#include <utility>
#include <boost/variant.hpp>

namespace fiveai
{

using MetricValue = boost::variant<bool, uint32_t, int32_t, uint64_t, int64_t, double, std::string>;

/**
 * @brief Formats metrics in InfluxDB line protocol format
 * @param measurement Name of the measurement
 * @param tags List of key-value tags (string/string pairs)
 * @param fields List of key-value fields (string/value pairs where value is any type convertible to ValueString)
 * @return Formatted line protocol
 */
std::string format_metrics(const std::string& measurement,
                           std::initializer_list<std::pair<std::string, std::string>> tags,
                           std::initializer_list<std::pair<std::string, MetricValue>> fields);

/**
 * @brief Log metrics to logger in the InfluxDB line protocol format
 * @param measurement Name of the measurement
 * @param topic Name of topic used to compare against enabled topics filter
 * @param tags List of key-value tags (string/string pairs)
 * @param fields List of key-value fields (string/value pairs where value is any type convertible to ValueString)
 */
ROSCPP_DECL void log_metrics(const std::string& measurement,
                 const std::string& topic,
                 std::initializer_list<std::pair<std::string, std::string>> tags,
                 std::initializer_list<std::pair<std::string, MetricValue>> fields);

/**
 * @brief Sets up the logger for message metrics
 */
void setup_metrics_logger();
}
