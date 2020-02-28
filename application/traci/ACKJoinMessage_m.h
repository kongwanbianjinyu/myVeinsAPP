//
// Generated file, do not edit! Created by nedtool 5.5 from veins/modules/application/traci/ACKJoinMessage.msg.
//

#ifndef __VEINS_ACKJOINMESSAGE_M_H
#define __VEINS_ACKJOINMESSAGE_M_H

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0505
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif

// dll export symbol
#ifndef VEINS_API
#  if defined(VEINS_EXPORT)
#    define VEINS_API  OPP_DLLEXPORT
#  elif defined(VEINS_IMPORT)
#    define VEINS_API  OPP_DLLIMPORT
#  else
#    define VEINS_API
#  endif
#endif

// cplusplus {{
#include "veins/base/utils/Coord.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/base/utils/SimpleAddress.h"
// }}


namespace veins {

/**
 * Class generated from <tt>veins/modules/application/traci/ACKJoinMessage.msg:31</tt> by nedtool.
 * <pre>
 * packet ACKJoinMessage extends BaseFrame1609_4
 * {
 *     LAddress::L2Type sourceCHId = -1;
 *     LAddress::L2Type targetCCMId = -1;
 * }
 * </pre>
 */
class VEINS_API ACKJoinMessage : public ::veins::BaseFrame1609_4
{
  protected:
    LAddress::L2Type sourceCHId;
    LAddress::L2Type targetCCMId;

  private:
    void copy(const ACKJoinMessage& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const ACKJoinMessage&);

  public:
    ACKJoinMessage(const char *name=nullptr, short kind=0);
    ACKJoinMessage(const ACKJoinMessage& other);
    virtual ~ACKJoinMessage();
    ACKJoinMessage& operator=(const ACKJoinMessage& other);
    virtual ACKJoinMessage *dup() const override {return new ACKJoinMessage(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual LAddress::L2Type& getSourceCHId();
    virtual const LAddress::L2Type& getSourceCHId() const {return const_cast<ACKJoinMessage*>(this)->getSourceCHId();}
    virtual void setSourceCHId(const LAddress::L2Type& sourceCHId);
    virtual LAddress::L2Type& getTargetCCMId();
    virtual const LAddress::L2Type& getTargetCCMId() const {return const_cast<ACKJoinMessage*>(this)->getTargetCCMId();}
    virtual void setTargetCCMId(const LAddress::L2Type& targetCCMId);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const ACKJoinMessage& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, ACKJoinMessage& obj) {obj.parsimUnpack(b);}

} // namespace veins

#endif // ifndef __VEINS_ACKJOINMESSAGE_M_H
