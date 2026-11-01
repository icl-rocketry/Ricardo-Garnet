#include "system.h"

#include <memory>
#include <string>

#include <libriccore/riccoresystem.h>

#include <HardwareSerial.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include "Config/general_config.h"

#include "Commands/commands.h"

#include "States/idle.h"



System::System():
RicCoreSystem(Commands::command_map,Commands::defaultEnabledCommands,Serial),
vspi(0)
{};


void System::systemSetup(){
    
    Serial.setRxBufferSize(GeneralConfig::SerialRxSize);
    Serial.begin(GeneralConfig::SerialBaud);
   
    //initialize statemachine with idle state
    statemachine.initalize(std::make_unique<Idle>(systemstatus,commandhandler));
    //any other setup goes here
    bldcSpin.setup();
    
    
};


void System::systemUpdate(){
    bldcSpin.update();
};