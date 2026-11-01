#pragma once

#include <libriccore/riccoresystem.h>

#include <memory>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"

#include "Commands/commands.h"

#include "BLDC/bldc.h"
#include "BLDC/OpenLoopSpin.h"

#include <SPI.h>
class System : public RicCoreSystem<System,SYSTEM_FLAG,Commands::ID>
{
    public:

        System();
        
        void systemSetup();

        void systemUpdate();

    private:

        SPIClass vspi;

        OpenLoopSpin bldcSpin; // swap for BLDCPhaseTest to run the single-phase test

};