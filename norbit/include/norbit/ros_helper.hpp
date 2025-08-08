#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <rclcpp/rclcpp.hpp>
#include "builtin_interfaces/msg/time.hpp"


/**
 * @brief Convert double timestamp to ROS header timestamp
 * @param[in] time_in_seconds The time in double
 * @return Header stamp in builtin_interfaces::msg::Time
 */ 
inline builtin_interfaces::msg::Time doubleToRosStamp(double time_in_seconds) {
    builtin_interfaces::msg::Time stamp;

    stamp.sec = static_cast<int32_t>(time_in_seconds);
    stamp.nanosec = static_cast<uint32_t>((time_in_seconds - stamp.sec) * 1e9);

    return stamp;
}

/**
 * @brief Convert ROS header timestamp to a double format timestamp
 * @param[in] stamp The time from header stamp
 * @return The double format time 
 */
inline double stampTodouble(const builtin_interfaces::msg::Time& stamp) {
    // Combine the seconds and nanoseconds into a single double
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

/**
 * @brief Convert boost timestamp to ROS header timestamp
 * @param[in] pt The time in boost format
 * @return Header stamp in builtin_interfaces::msg::Time
 */ 
inline builtin_interfaces::msg::Time boostToRosStamp(const boost::posix_time::ptime& pt) {
    // Define the UNIX epoch as a boost::ptime
    static const boost::posix_time::ptime epoch(
        boost::gregorian::date(1970, 1, 1));

    // Calculate time duration from epoch
    boost::posix_time::time_duration diff = pt - epoch;

    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(diff.total_seconds());
    stamp.nanosec = static_cast<uint32_t>((diff.total_microseconds() % 1000000) * 1000);

    return stamp;
}

/**
 * @brief Convert two header stamp to a double format duration
 * @param[in] start The start time in Header stamp, which is builtin_interfaces::msg::Time
 * @param[in] end The end time in Header stamp, which is builtin_interfaces::msg::Time
 * @return The double format time duration 
 */ 
inline double durationFromStamp(
    const builtin_interfaces::msg::Time& start,
    const builtin_interfaces::msg::Time& end)
{
    rclcpp::Time t_start(start);
    rclcpp::Time t_end(end);

    rclcpp::Duration duration = t_end - t_start;
    return duration.seconds();  
}
