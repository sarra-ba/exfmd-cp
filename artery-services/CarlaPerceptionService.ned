package artery.application;

simple CarlaPerceptionService like artery.application.ItsG5Service
{
    parameters:
        @class(CarlaPerceptionService);
        double maxInterval @unit(s) = default(1s);
        bool fixedRate = default(true);
        string carlaJsonPath = default("/home/syfra/artery_fresh/scenarios/F2MD_Highway_CPM/carla_perceived_objects.json");
}
