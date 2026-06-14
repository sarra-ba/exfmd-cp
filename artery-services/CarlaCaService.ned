package artery.application;

simple CarlaCaService like ItsG5Service
{
    parameters:
        double maxInterval @unit(s) = default(0.1s);
        string shmPath = default("/dev/shm/hero_state");
}
