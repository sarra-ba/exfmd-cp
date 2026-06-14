#ifndef ARTERY_CARLAPERCEPTIONSERVICE_H_
#define ARTERY_CARLAPERCEPTIONSERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <omnetpp/simtime.h>
#include <string>
#include <vector>

namespace artery
{

using namespace omnetpp;
class Timer;

struct CarlaObject {
    double x, y, z, distance, velocity;
    int point_count;
};

class CarlaPerceptionService : public ItsG5BaseService
{
public:
    void initialize() override;
    void trigger() override;
    void indicate(const vanetza::btp::DataIndication&,
                  std::unique_ptr<vanetza::UpPacket>) override;
    const Timer* getTimer() const;
    void sendCpm(const omnetpp::SimTime&);

private:
    std::vector<CarlaObject> readCarlaObjects();
    vanetza::asn1::Cpm buildCpm(const std::vector<CarlaObject>&);

    const VehicleDataProvider* mVehicleDataProvider = nullptr;
    const Timer* mTimer = nullptr;
    omnetpp::SimTime mLastCpmTimestamp;
    omnetpp::SimTime mGenCpmMax;
    bool mFixedRate;
    std::string mJsonPath;
};

} // namespace artery
#endif
