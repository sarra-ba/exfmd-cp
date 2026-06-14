#include "CarlaPerceptionService.h"
#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/CpmObject.h"
#include "artery/utility/simtime_cast.h"
#include "Timer.h"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <omnetpp/cexception.h>
#include <boost/units/systems/si/prefixes.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <regex>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>

namespace artery
{

using namespace omnetpp;

static const auto centimeter_per_second =
    vanetza::units::si::meter_per_second * boost::units::si::centi;

Define_Module(CarlaPerceptionService)

void CarlaPerceptionService::initialize()
{
    ItsG5BaseService::initialize();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mTimer = &getFacilities().get_const<Timer>();
    mGenCpmMax = par("maxInterval");
    mFixedRate = par("fixedRate");
    mJsonPath = par("carlaJsonPath").stdstringValue();
    mLastCpmTimestamp = simTime();
    std::cout << "CarlaPerceptionService initialized, json=" << mJsonPath << std::endl;
}

void CarlaPerceptionService::trigger()
{
    const SimTime T_elapsed = simTime() - mLastCpmTimestamp;
    if (T_elapsed >= mGenCpmMax) {
        sendCpm(simTime());
    }
}

void CarlaPerceptionService::indicate(
    const vanetza::btp::DataIndication& indication,
    std::unique_ptr<vanetza::UpPacket> packet)
{
    Asn1PacketVisitor<vanetza::asn1::Cpm> visitor;
    const vanetza::asn1::Cpm* cpm = boost::apply_visitor(visitor, *packet);
    if (!cpm || !cpm->validate()) return;

    const vanetza::asn1::Cpm& msg = *cpm;
    if (msg->cpm.cpmParameters.numberOfPerceivedObjects == 0) return;

    const PerceivedObjectContainer* poc =
        msg->cpm.cpmParameters.perceivedObjectContainer;
    if (!poc) return;

    std::string json = "{\"sender\":" +
        std::to_string(msg->header.stationID) +
        ",\"simtime\":" + std::to_string(simTime().dbl()) +
        ",\"objects\":[";

    for (int i = 0; i < poc->list.count; i++) {
        PerceivedObject_t* po = poc->list.array[i];
        if (i > 0) json += ",";
        json += "{\"id\":" + std::to_string(po->objectID) +
                ",\"x\":" + std::to_string(po->xDistance.value / 100.0) +
                ",\"y\":" + std::to_string(po->yDistance.value / 100.0) +
                ",\"vx\":" + std::to_string(po->xSpeed.value / 100.0) +
                "}";
    }
    json += "],\"count\":" + std::to_string(poc->list.count) + "}";

    std::string rxPath = mJsonPath.substr(0, mJsonPath.rfind('/')) +
        "/carla_received_cpm.json";
    std::ofstream f(rxPath);
    if (f.is_open()) {
        f << json;
        f.close();
        std::cout << "CarlaPerceptionService: received CPM from "
                  << msg->header.stationID
                  << " with " << poc->list.count
                  << " objects at " << SIMTIME_STR(simTime()) << std::endl;
    }
}

std::vector<CarlaObject> CarlaPerceptionService::readCarlaObjects()
{
    std::vector<CarlaObject> objects;
    std::ifstream file(mJsonPath);
    if (!file.is_open()) return objects;
    std::stringstream buf;
    buf << file.rdbuf();
    std::string content = buf.str();

    std::regex obj_regex(
        R"(\{[^}]*"x"\s*:\s*([-\d.e+]+)[^}]*"y"\s*:\s*([-\d.e+]+)[^}]*"z"\s*:\s*([-\d.e+]+)[^}]*"distance"\s*:\s*([-\d.e+]+)[^}]*"point_count"\s*:\s*(\d+)[^}]*"velocity"\s*:\s*([-\d.e+]+))");

    auto begin = std::sregex_iterator(content.begin(), content.end(), obj_regex);
    auto end   = std::sregex_iterator();
    for (auto it = begin; it != end; ++it) {
        CarlaObject obj;
        obj.x           = std::stod((*it)[1].str());
        obj.y           = std::stod((*it)[2].str());
        obj.z           = std::stod((*it)[3].str());
        obj.distance    = std::stod((*it)[4].str());
        obj.point_count = std::stoi((*it)[5].str());
        obj.velocity    = std::stod((*it)[6].str());
        objects.push_back(obj);
    }
    return objects;
}

void CarlaPerceptionService::sendCpm(const SimTime& T_now)
{
    auto objects = readCarlaObjects();
    auto cpm = buildCpm(objects);
    mLastCpmTimestamp = T_now;

    using namespace vanetza;
    btp::DataRequestB request;
    request.destination_port = btp::ports::CPM;
    request.gn.its_aid = aid::CP;
    request.gn.transport_type = geonet::TransportType::SHB;
    request.gn.maximum_lifetime =
        geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
    request.gn.traffic_class.tc_id(
        static_cast<unsigned>(dcc::Profile::DP2));
    request.gn.communication_profile =
        geonet::CommunicationProfile::ITS_G5;

    CpmObject obj(std::move(cpm));
    using CpmByteBuffer = convertible::byte_buffer_impl<asn1::Cpm>;
    std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
    std::unique_ptr<convertible::byte_buffer> buffer {
        new CpmByteBuffer(obj.shared_ptr()) };
    payload->layer(OsiLayer::Application) = std::move(buffer);
    this->request(request, std::move(payload));

    std::cout << "CarlaPerceptionService: sent CPM with "
              << objects.size() << " CARLA objects at "
              << SIMTIME_STR(simTime()) << std::endl;
}

vanetza::asn1::Cpm CarlaPerceptionService::buildCpm(
    const std::vector<CarlaObject>& objects)
{
    vanetza::asn1::Cpm message;
    ItsPduHeader_t& header = (*message).header;
    header.protocolVersion = 2;
    header.messageID = 14;
    header.stationID = mVehicleDataProvider->getStationId() % 4294967295;

    long referenceTime = countTaiMilliseconds(mTimer->getCurrentTime());
    (*message).cpm.generationDeltaTime =
        (GenerationDeltaTime_t)(referenceTime % 65536);

    CpmManagementContainer_t& mgmt =
        (*message).cpm.cpmParameters.managementContainer;
    mgmt.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    mgmt.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    long mlat = std::round(mVehicleDataProvider->latitude().value() * 1e7);
    long mlon = std::round(mVehicleDataProvider->longitude().value() * 1e7);
    mgmt.referencePosition.latitude =
        (mlat > -900000000 && mlat < 900000001) ? mlat : Latitude_unavailable;
    mgmt.referencePosition.longitude =
        (mlon > -1800000000 && mlon < 1800000001) ? mlon : Longitude_unavailable;
    mgmt.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
        SemiAxisLength_unavailable;
    mgmt.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
        SemiAxisLength_unavailable;
    mgmt.referencePosition.positionConfidenceEllipse.semiMajorOrientation =
        HeadingValue_unavailable;
    mgmt.stationType = StationType_passengerCar;

    (*message).cpm.cpmParameters.stationDataContainer =
        vanetza::asn1::allocate<StationDataContainer_t>();
    (*message).cpm.cpmParameters.stationDataContainer->present =
        StationDataContainer_PR_originatingVehicleContainer;
    OriginatingVehicleContainer_t& ovc =
        (*message).cpm.cpmParameters.stationDataContainer
            ->choice.originatingVehicleContainer;
    auto headingVal = std::round(
        mVehicleDataProvider->heading().value() * 1800.0 / M_PI);
    ovc.heading.headingValue =
        (headingVal >= 0 && headingVal <= 3600)
            ? (HeadingValue_t)headingVal : HeadingValue_unavailable;
    ovc.heading.headingConfidence = HeadingConfidence_unavailable;
    ovc.speed.speedValue          = SpeedValue_unavailable;
    ovc.speed.speedConfidence     = SpeedConfidence_unavailable;
    ovc.driveDirection            = DriveDirection_forward;

    if (!objects.empty()) {
        (*message).cpm.cpmParameters.perceivedObjectContainer =
            vanetza::asn1::allocate<PerceivedObjectContainer_t>();
        PerceivedObjectContainer_t& poc =
            *(*message).cpm.cpmParameters.perceivedObjectContainer;

        int objID = 1;
        int maxObj = 10; int objCount = 0;
        for (const auto& obj : objects) {
            if (objCount++ >= maxObj) break;
            PerceivedObject_t* po =
                vanetza::asn1::allocate<PerceivedObject_t>();
            po->objectID = objID++;
            po->timeOfMeasurement = TimeOfMeasurement_oneMilliSecond;
            double dx = std::max(-132768.0, std::min(132767.0, obj.x * 100.0));
            double dy = std::max(-132768.0, std::min(132767.0, obj.y * 100.0));
            po->xDistance.value      = (long)dx;
            po->xDistance.confidence = DistanceConfidence_unavailable;
            po->yDistance.value      = (long)dy;
            po->yDistance.confidence = DistanceConfidence_unavailable;
            long vel_cms = (long)(std::abs(obj.velocity) * 100.0);
            vel_cms = std::max(0L, std::min(16382L, vel_cms));
            po->xSpeed.value      = vel_cms;
            po->xSpeed.confidence = SpeedConfidence_unavailable;
            po->ySpeed.value      = SpeedValueExtended_unavailable;
            po->ySpeed.confidence = SpeedConfidence_unavailable;
            ASN_SEQUENCE_ADD(&poc, po);
        }
    }

    (*message).cpm.cpmParameters.numberOfPerceivedObjects =
        (*message).cpm.cpmParameters.perceivedObjectContainer
            ? (*message).cpm.cpmParameters.perceivedObjectContainer->list.count
            : 0;

    return message;
}

const Timer* CarlaPerceptionService::getTimer() const
{
    return mTimer;
}

} // namespace artery
