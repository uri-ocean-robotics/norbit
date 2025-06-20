#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <rclcpp/rclcpp.hpp>
#include "builtin_interfaces/msg/time.hpp"

// double time to stamp (builtin_interfaces::msg::Time)
inline builtin_interfaces::msg::Time doubleToRosStamp(double time_in_seconds) {
    builtin_interfaces::msg::Time stamp;

    stamp.sec = static_cast<int32_t>(time_in_seconds);
    stamp.nanosec = static_cast<uint32_t>((time_in_seconds - stamp.sec) * 1e9);

    return stamp;
}

// boost time to stamp (builtin_interfaces::msg::Time)
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

// returns duration in seconds as double from two stamp (builtin_interfaces::msg::Time)
inline double durationFromStamp(
    const builtin_interfaces::msg::Time& start,
    const builtin_interfaces::msg::Time& end)
{
    rclcpp::Time t_start(start);
    rclcpp::Time t_end(end);

    rclcpp::Duration duration = t_end - t_start;
    return duration.seconds();  
}

inline double stampTodouble(const builtin_interfaces::msg::Time& stamp) {
    // Combine the seconds and nanoseconds into a single double
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}
