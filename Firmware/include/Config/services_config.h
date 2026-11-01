#pragma once

#include <stdint.h>


namespace Services{
    /**
     * @brief ID of user defined services NB start at 2, all ID's below 2 are reserved for default services found in 'rnp_networkmanager.h'
     * 
     */
    enum class ID:uint8_t{
        Serivce1 = 3
    };

};