/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#pragma once

#include <functional>
#include <map>

namespace twist {

/**
 * @ingroup common
 * Namespace for common utilities
 */
namespace common {

/**
 * Convert enumerations
 * @tparam In type to convert
 * @tparam Out type of conversion
 * @param enumMap map of inputs to convert
 * @return converted value
 */
template<typename In, typename Out>
[[nodiscard]] auto enumMapper(std::map<In, Out> enumMap) noexcept -> std::function<Out(In)> {
    return [enumMap = std::move(enumMap)](In in) { return enumMap.at(in); };
}

}  // namespace common
}  // namespace twist
