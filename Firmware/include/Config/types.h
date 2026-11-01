#pragma once

#include <libriccore/riccoretypes.h>

//we need the forward declarations for the system aswell
#include "forward_decl.h"

#include "systemflags_config.h"
#include "commands_config.h"

namespace Types{
    using CoreTypes = RicCoreTypes<ForwardDecl_SystemClass,SYSTEM_FLAG,Commands::ID,256>;
    //any other useful aliases used in multiple places should be defined here
};

