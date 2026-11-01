#pragma once
#include <stdint.h>

#include "Config/forward_decl.h"
#include <librnp/rnp_packet.h>

namespace Commands{
    
    void FreeRamCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);

}