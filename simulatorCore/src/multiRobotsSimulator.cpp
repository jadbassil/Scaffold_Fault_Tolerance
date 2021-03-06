/*
 * @file multiRobotsSimulator.cpp
 *
 *  Created on: 14/07/16
 *      Author: pthalamy
 */

#include <iostream>
#include <string.h>

#include "multiRobotsSimulator.h"
#include "trace.h"

using namespace std;

namespace MultiRobots {

MultiRobotsSimulator::MultiRobotsSimulator(int argc, char *argv[], BlockCodeBuilder bcb)
    : BaseSimulator::Simulator(argc, argv, bcb) {
#ifdef DEBUG_OBJECT_LIFECYCLE
    OUTPUT << TermColor::LifecycleColor << "MultiRobotsSimulator constructor" << TermColor::Reset << endl;
#endif
}

MultiRobotsSimulator::~MultiRobotsSimulator() {
#ifdef DEBUG_OBJECT_LIFECYCLE
    OUTPUT << TermColor::LifecycleColor << "MultiRobotsSimulator destructor" << TermColor::Reset <<endl;
#endif
}

void MultiRobotsSimulator::createSimulator(int argc, char *argv[], BlockCodeBuilder bcb) {
    simulator =  new MultiRobotsSimulator(argc, argv, bcb);
    simulator->parseConfiguration(argc, argv);
    simulator->startSimulation();
}

void MultiRobotsSimulator::loadWorld(const Cell3DPosition &gridSize, const Vector3D &gridScale,
                                     int argc, char *argv[]) {
    world = new MultiRobotsWorld(gridSize, gridScale, argc,argv);

    if (GlutContext::GUIisEnabled)
        world->loadTextures("../../simulatorCore/resources/textures/latticeTextures");

    World::setWorld(world);
}

void MultiRobotsSimulator::loadBlock(TiXmlElement *blockElt, bID blockId, BlockCodeBuilder bcb,
                                      const Cell3DPosition &pos, const Color &color, bool master) {

    // Any additional configuration file parsing exclusive to this type of block should be performed
    //  here, using the blockElt TiXmlElement.

    // ...Parsing code...

    // Finally, add block to the world
    ((MultiRobotsWorld*)world)->addBlock(blockId, bcb, pos, color, 0, master);
    world->getBlockById(blockId)->blockCode->parseUserBlockElements(blockElt);
}

} // MultiRobots namespace
