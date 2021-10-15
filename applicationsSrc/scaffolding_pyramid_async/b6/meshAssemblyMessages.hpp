#ifndef MC3D_MESSAGES_H_
#define MC3D_MESSAGES_H_

#include "network.h"

#include "meshRuleMatcher.hpp"

static const uint MSG_DELAY = 0;

using namespace MeshCoating;

class RequestTargetCellMessage : public HandleableMessage {
    const Cell3DPosition srcPos;
    bID srcId;
public:
    RequestTargetCellMessage(const Cell3DPosition& _srcPos, bID _srcId)
        : HandleableMessage(), srcPos(_srcPos), srcId(_srcId) {};
    virtual ~RequestTargetCellMessage() {};

    virtual void handle(BaseSimulator::BlockCode*) override;
    virtual Message* clone() const override { return new RequestTargetCellMessage(*this); }
    virtual string getName() const override { return "RequestTargetCell{" + srcPos.to_string()
            + ", " + to_string(srcId) + "}"; }
};

class ProvideTargetCellMessage : public HandleableMessage {
    const Cell3DPosition tPos;
    const Cell3DPosition dstPos;
public:
    ProvideTargetCellMessage(const Cell3DPosition& _tPos, const Cell3DPosition& _dstPos)
        : HandleableMessage(), tPos(_tPos), dstPos(_dstPos) {};
    virtual ~ProvideTargetCellMessage() {};

    virtual void handle(BaseSimulator::BlockCode*) override;
    virtual Message* clone() const override { return new ProvideTargetCellMessage(*this); }
    virtual string getName() const override { return "ProvideTargetCell{" + tPos.to_string()
            + ", " + dstPos.to_string() + "}"; }
};

/**
 * Sent by a coordinator that has just arrived in order to ask waiting module to
 *  resend their RequestTargetCell
 */
class CoordinatorReadyMessage : public HandleableMessage {
public:
    CoordinatorReadyMessage()
        : HandleableMessage() {};
    virtual ~CoordinatorReadyMessage() {};

    virtual void handle(BaseSimulator::BlockCode*) override;
    virtual Message* clone() const override { return new CoordinatorReadyMessage(*this); }
    virtual string getName() const override { return "CoordinatorReady"; }
};

class TileNotReadyMessage : public HandleableMessage {
    const Cell3DPosition dstPos;
public:
    TileNotReadyMessage(const Cell3DPosition& _dstPos)
        : HandleableMessage(), dstPos(_dstPos) {};
    virtual ~TileNotReadyMessage() {};

    virtual void handle(BaseSimulator::BlockCode*) override;
    virtual Message* clone() const override { return new TileNotReadyMessage(*this); }
    virtual string getName() const override { return "TileNotReady"; }
};

class TileInsertionReadyMessage : public HandleableMessage {
public:
    TileInsertionReadyMessage() : HandleableMessage() {};
    virtual ~TileInsertionReadyMessage() {};

    virtual void handle(BaseSimulator::BlockCode*) override;
    virtual Message* clone() const override { return new TileInsertionReadyMessage(*this); }
    virtual string getName() const override { return "TileInsertionReady"; }
};

/////////////////////////////////////////////////////////////////
///////////////////// MOTION COORDINATION ///////////////////////
/////////////////////////////////////////////////////////////////

/**
 * This message should be routed through the line until it reaches the dstPos, which as a pivot
 *  module must check whether its light is on and either give a go, or wait until its status
 *  clears and send a go at that time.
 */

// BRIDGING
//TODO PERLA...............................................................................///
// Create a class for BridgingPosition


class ReachBridgingPosition : public HandleableMessage {
    const Cell3DPosition srcPos;
    const Cell3DPosition dstPos;
    const int brokenInterfaceID;
public:
    ReachBridgingPosition(const Cell3DPosition& _srcPos, const Cell3DPosition& _dstPos, int _brokenInterfaceID)
        : HandleableMessage(), srcPos(_srcPos), dstPos(_dstPos), brokenInterfaceID(_brokenInterfaceID) {};
    virtual ~ReachBridgingPosition() {};

    virtual void handle(BaseSimulator::BlockCode*) override;
    virtual Message* clone() const override { return new ReachBridgingPosition(*this); }
    virtual string getName() const override { return "ReachBridgingPosition{" + srcPos.to_string()
            + ", " + dstPos.to_string() + "}"; }
};

class HelperPositionReachedMessage : public HandleableMessage {
    //const Cell3DPosition bridgePos;
    const Cell3DPosition targetPosition;

    const int brokenInterfaceID;
public:
    HelperPositionReachedMessage(int _brokenInterfaceID, Cell3DPosition  _targetPosition)
        : brokenInterfaceID(_brokenInterfaceID) ,targetPosition(_targetPosition) {};
    
    virtual ~HelperPositionReachedMessage() {};

    virtual void handle(BaseSimulator::BlockCode*) override;
    virtual Message* clone() const override { return new HelperPositionReachedMessage(*this); }
    virtual string getName() const override { return "HelperPositionReached{" + targetPosition.to_string() +"}";
    }
};

//Request additional module

class RequestAdditionalModule : public HandleableMessage {
   MeshComponent epl;
public:
    RequestAdditionalModule()
        : HandleableMessage(){};
    RequestAdditionalModule(MeshComponent _epl)
        : HandleableMessage(), epl(_epl){};
   
    virtual ~RequestAdditionalModule() {};
    virtual void handle(BaseSimulator::BlockCode*) override;
    virtual Message* clone() const override { return new RequestAdditionalModule(*this); }
    virtual string getName() const override { return "RequestAdditionalModule{}";};

};

//TODAY
class PositionToSwapMessage: public HandleableMessage {
    Cell3DPosition targetPosition;
    Cell3DPosition srcPos, dstPos;
    P2PNetworkInterface itf;
    public:
    PositionToSwapMessage(Cell3DPosition  _targetPosition, Cell3DPosition _srcPos, Cell3DPosition _dstPos, P2PNetworkInterface _itf)
     : targetPosition(_targetPosition), srcPos(_srcPos), dstPos(_dstPos), itf(_itf) {};
    
    virtual ~PositionToSwapMessage() {};

    virtual void handle(BaseSimulator::BlockCode*) override;
    virtual Message* clone() const override { return new PositionToSwapMessage(*this); }
    virtual string getName() const override { return "PositionToSwap{" + targetPosition.to_string() +"}";
    }

};

//..................................................................................................................//

class ProbePivotLightStateMessage : public HandleableMessage {
    const Cell3DPosition srcPos;
    const Cell3DPosition targetPos; // Next position the module is seeking to reach
    Cell3DPosition finalTargetPos; // the final position to be reached by the module
    /**
     * Final component to be reached by a series of intermediate motions such as this one
     */
    const MeshComponent finalComponent;

public:
    ProbePivotLightStateMessage(const Cell3DPosition& _srcPos,
                                const Cell3DPosition& _targetPos,
                                const MeshComponent _finalComponent,
                                Cell3DPosition& _finalPos)
        : HandleableMessage(), srcPos(_srcPos),
          targetPos(_targetPos), finalTargetPos (_finalPos), finalComponent(_finalComponent) {};

// public:
//     ProbePivotLightStateMessage(const Cell3DPosition& _srcPos,
//                                 const Cell3DPosition& _targetPos,
//                                 const MeshComponent _finalComponent)
//         : HandleableMessage(), srcPos(_srcPos),
//           targetPos(_targetPos),finalComponent(_finalComponent) {};
    
    virtual ~ProbePivotLightStateMessage() {};

    virtual void handle(BaseSimulator::BlockCode*) override;
    virtual Message* clone() const override { return new ProbePivotLightStateMessage(*this); }
    virtual string getName() const override { return "ProbePivotLightState{" + srcPos.to_string()
            + ", " + targetPos.to_string()
            +  ", " + finalTargetPos.to_string()
            + "}";
    }
};


/**
 * This message is routed back from the pivot module to the module awaiting motion
 *  in order to notify it that it can be safely performed.
 */
class GreenLightIsOnMessage : public HandleableMessage {
    const Cell3DPosition srcPos;
    const Cell3DPosition dstPos;
    Cell3DPosition newTargetPosition;
public:
    // GreenLightIsOnMessage(const Cell3DPosition& _srcPos,
    //                       const Cell3DPosition& _dstPos)
    //     : HandleableMessage(), srcPos(_srcPos), dstPos(_dstPos) {
    //         newTargetPosition = Cell3DPosition(-1,-1,-1);
    //     };

public:
    GreenLightIsOnMessage(const Cell3DPosition& _srcPos,
                          const Cell3DPosition& _dstPos)
        : HandleableMessage(), srcPos(_srcPos), dstPos(_dstPos){
            newTargetPosition = Cell3DPosition(-1,-1,-1);
    };

    GreenLightIsOnMessage(const Cell3DPosition& _srcPos,
                          const Cell3DPosition& _dstPos, Cell3DPosition _newTargetPosition)
        : HandleableMessage(), srcPos(_srcPos), dstPos(_dstPos), newTargetPosition(_newTargetPosition) {
    };
    virtual ~GreenLightIsOnMessage() {};

    virtual void handle(BaseSimulator::BlockCode*) override;
    virtual Message* clone() const override { return new GreenLightIsOnMessage(*this); }
    virtual string getName() const override { return "GreenLightIsOn{" + srcPos.to_string()
            + ", " + dstPos.to_string() + ", " + newTargetPosition.to_string() + "}";
    }
};


/**
 * This message is sent to a RED light pivot when a module it actuated has
 *  reached its final rotation in the scaffold, instructing it to turn back green.
 */
class FinalTargetReachedMessage : public HandleableMessage {
    const Cell3DPosition finalPos;
public:
    FinalTargetReachedMessage(const Cell3DPosition& _finalPos)
        : HandleableMessage(), finalPos(_finalPos) {};
    virtual ~FinalTargetReachedMessage() {};

    virtual void handle(BaseSimulator::BlockCode*) override;
    virtual Message* clone() const override { return new FinalTargetReachedMessage(*this); }
    virtual string getName() const override { return "FinalTargetReached{" + finalPos.to_string() +"}";
    }
};
#endif /* MC3D_MESSAGES_H_ */
