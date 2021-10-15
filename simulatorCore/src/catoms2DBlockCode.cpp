/*
 * catoms2DBlockCode.cpp
 *
 *  Created on: 12 janvier 2014
 *      Author: Benoît
 */

#include <iostream>
#include "catoms2DBlockCode.h"
#include "trace.h"
#include "uniqueEventsId.h"
#include "events.h"
#include "scheduler.h"

using namespace std;

namespace Catoms2D {

Catoms2DBlockCode::Catoms2DBlockCode(Catoms2DBlock *host):BlockCode(host) {
#ifdef DEBUG_OBJECT_LIFECYCLE
    OUTPUT << "Catoms2DBlockCode constructor" << endl;
#endif
}

Catoms2DBlockCode::~Catoms2DBlockCode() {
#ifdef DEBUG_OBJECT_LIFECYCLE
    OUTPUT << "Catoms2DBlockCode destructor" << endl;
#endif
}

void Catoms2DBlockCode::processLocalEvent(EventPtr pev) {
    MessagePtr message;
    stringstream info;

    BlockCode::processLocalEvent(pev);

    switch (pev->eventType) {
        case EVENT_ROTATION2D_END: {

            info.str("");
            info << " finished rotating to " << hostBlock->position;
            scheduler->trace(info.str(),hostBlock->blockId, GOLD);

            onMotionEnd();
        }  break;
    }
}

}
