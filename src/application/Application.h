/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#pragma once

#include <memory>

#include "business/flow/InitManager.h"

/**
 * @ingroup twist
 *
 * Namespace for all TWIST modules
 */
namespace twist {

class Application final {
public:
    /**
     * Constructor
     *
     * @param nodeHandle node handle from main
     */
    explicit Application(ros::NodeHandle& nodeHandle);

    /**
     * Destructor default
     */
    ~Application() = default;

private:
    std::unique_ptr<business::InitManager> initManager;
};

}  // namespace twist

