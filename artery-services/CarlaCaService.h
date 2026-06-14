#ifndef ARTERY_CARLACASERVICE_H_
#define ARTERY_CARLACASERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/CaService.h"
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/asn1/cam.hpp>
#include <omnetpp/simtime.h>

namespace artery
{

class Timer;

struct HeroState {
    double latitude;
    double longitude;
    double heading;
    double speed;
    double acceleration;
    double yaw_rate;
    double timestamp;
};

class CarlaCaService : public ItsG5BaseService
{
public:
    void initialize() override;
    void trigger() override;
    void indicate(const vanetza::btp::DataIndication&,
                  std::unique_ptr<vanetza::UpPacket>) override;

private:
    bool readHeroState(HeroState&);
    void sendCam(const omnetpp::SimTime&);

    const VehicleDataProvider* mVehicleDataProvider = nullptr;
    const Timer* mTimer = nullptr;
    omnetpp::SimTime mLastCamTimestamp;
    omnetpp::SimTime mGenCamMax;
    std::string mShmPath;
};

} // namespace artery
#endif
