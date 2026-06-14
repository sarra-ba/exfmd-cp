#include "CarlaCaService.h"
#include "artery/application/CaService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/CaObject.h"
#include "artery/utility/simtime_cast.h"
#include "Timer.h"
#include <vanetza/btp/ports.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <omnetpp/cexception.h>
#include <unistd.h>
#include <boost/units/systems/si/prefixes.hpp>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>

namespace artery
{

using namespace omnetpp;

auto microdegree_ca = vanetza::units::degree * boost::units::si::micro;
auto decidegree_ca  = vanetza::units::degree * boost::units::si::deci;
auto centimeter_per_second_ca = vanetza::units::si::meter_per_second * boost::units::si::centi;

Define_Module(CarlaCaService)

void CarlaCaService::initialize()
{
    ItsG5BaseService::initialize();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mTimer = &getFacilities().get_const<Timer>();
    mGenCamMax = par("maxInterval");
    std::string nodeName = getParentModule()->getParentModule()->getName();
    int nodeIdx = 0;
    std::string digits = "";
    for (char c : nodeName) { if (isdigit(c)) digits += c; }
    if (!digits.empty()) nodeIdx = std::stoi(digits);
    mShmPath = "/dev/shm/agent_" + std::to_string(nodeIdx);
        mLastCamTimestamp = simTime();
    std::cout << "CarlaCaService initialized, shm=" << mShmPath << std::endl;
}

void CarlaCaService::trigger()
{ 
    const SimTime T_elapsed = simTime() - mLastCamTimestamp;
    usleep(30000); // 30ms
    if (T_elapsed >= mGenCamMax) {
        sendCam(simTime());
    }
}

void CarlaCaService::indicate(
    const vanetza::btp::DataIndication&,
    std::unique_ptr<vanetza::UpPacket>)
{
    std::cout << "CarlaCaService: received CAM from another vehicle at " << simTime() << std::endl;
}

bool CarlaCaService::readHeroState(HeroState& state)
{
    int fd = open(mShmPath.c_str(), O_RDONLY);
    if (fd < 0) return false;
    void* ptr = mmap(nullptr, sizeof(HeroState), PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (ptr == MAP_FAILED) return false;
    memcpy(&state, ptr, sizeof(HeroState));
    munmap(ptr, sizeof(HeroState));
    return true;
}

void CarlaCaService::sendCam(const SimTime& T_now)
{
    HeroState state;
    if (!readHeroState(state)) {
        std::cout << "CarlaCaService: failed to read shm" << std::endl;
        return;
    }

    vanetza::asn1::Cam message;
    ItsPduHeader_t& header = (*message).header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_cam;
    header.stationID = mVehicleDataProvider->station_id();

    CoopAwareness_t& cam = (*message).cam;
    long genDeltaTime = countTaiMilliseconds(mTimer->getCurrentTime()) % 65536;
    cam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;

    // Basic container
    BasicContainer_t& basic = cam.camParameters.basicContainer;
    basic.stationType = StationType_passengerCar;
    basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;

    long lat = std::round(state.latitude  * 1e7);
    long lon = std::round(state.longitude * 1e7);
    basic.referencePosition.latitude  = (lat > -900000000 && lat < 900000001)   ? lat : Latitude_unavailable;
    basic.referencePosition.longitude = (lon > -1800000000 && lon < 1800000001) ? lon : Longitude_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence  = SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence  = SemiAxisLength_unavailable;

    // High frequency container
    HighFrequencyContainer_t& hfc = cam.camParameters.highFrequencyContainer;
    hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
    BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;

    long headingVal = std::round(state.heading * 10.0);
    bvc.heading.headingValue = (headingVal >= 0 && headingVal <= 3600) ? headingVal : HeadingValue_unavailable;
    bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;

    long speedVal = std::round(state.speed * 100.0);
    bvc.speed.speedValue = (speedVal >= 0 && speedVal <= 16382) ? speedVal : SpeedValue_unavailable;
    bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;

    bvc.driveDirection = state.speed >= 0.0 ? DriveDirection_forward : DriveDirection_backward;

    double accel = state.acceleration;
    if (accel >= -160.0 && accel <= 161.0) {
        bvc.longitudinalAcceleration.longitudinalAccelerationValue =
            accel * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
    } else {
        bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
    }
    bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;

    long yawVal = std::round(state.yaw_rate * 180.0 / M_PI * 100.0);
    bvc.yawRate.yawRateValue = (yawVal >= -32766 && yawVal <= 32766) ? yawVal : YawRateValue_unavailable;

    bvc.curvature.curvatureValue = CurvatureValue_unavailable;
    bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
    bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
    bvc.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_noTrailerPresent;
    bvc.vehicleWidth = VehicleWidth_unavailable;

    std::string error;
    if (!message.validate(error)) {
        std::cout << "CarlaCaService: invalid CAM: " << error << std::endl;
        return;
    }

    /* Encoder CAM en ASN.1 et envoyer via UDP (ports 9101-9103) */
    {
        vanetza::ByteBuffer encoded = message.encode();
        if (!encoded.empty()) {
            int nodeIdx = mShmPath.back() - '0';
            int udp_port = 9101 + nodeIdx;
            int sock = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock >= 0) {
                struct sockaddr_in addr{};
                addr.sin_family = AF_INET;
                addr.sin_port = htons(udp_port);
                addr.sin_addr.s_addr = inet_addr("127.0.0.1");
                sendto(sock, encoded.data(), encoded.size(), 0,
                       (struct sockaddr*)&addr, sizeof(addr));
                close(sock);
                std::cout << "CarlaCaService: sent ASN.1 CAM " << encoded.size() << " bytes on port " << udp_port << std::endl;
            }
        }
    }

    mLastCamTimestamp = T_now;

    using namespace vanetza;
    btp::DataRequestB request;
    request.destination_port = btp::ports::CAM;
    request.gn.its_aid = aid::CA;
    request.gn.transport_type = geonet::TransportType::SHB;
    request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
    request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    CaObject obj(std::move(message));
    using CamByteBuffer = convertible::byte_buffer_impl<asn1::Cam>;
    std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
    std::unique_ptr<convertible::byte_buffer> buffer { new CamByteBuffer(obj.shared_ptr()) };
    payload->layer(OsiLayer::Application) = std::move(buffer);
    this->request(request, std::move(payload));

    /* UDP → ROS2 bridge */
    {
        int nodeIdx = mShmPath.back() - '0';
        int udp_port = 9001 + nodeIdx;
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock >= 0) {
            struct sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_port = htons(udp_port);
            addr.sin_addr.s_addr = inet_addr("127.0.0.1");
            char buf[256];
            snprintf(buf, sizeof(buf),
                "{\"lat\":%.7f,\"lon\":%.7f,\"heading\":%.4f,\"speed\":%.4f,\"accel\":%.4f,\"yaw_rate\":%.4f,\"station_id\":%u}",
                state.latitude, state.longitude, state.heading, state.speed,
                state.acceleration, state.yaw_rate, header.stationID);
            sendto(sock, buf, strlen(buf), 0, (struct sockaddr*)&addr, sizeof(addr));
            close(sock);
        }
    }

    std::cout << "CarlaCaService: sent CAM hero lat=" << state.latitude
              << " lon=" << state.longitude
              << " speed=" << state.speed
              << " heading=" << state.heading
              << " at " << SIMTIME_STR(T_now) << std::endl;
}

} // namespace artery
