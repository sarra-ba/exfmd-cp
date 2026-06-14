FROM carlasim/carla:0.9.16
USER root
WORKDIR /workspace

# Integrer la piste UPHF dans l'image
COPY uphf_gyrovia.tar.gz /tmp/uphf_gyrovia.tar.gz
RUN cd /workspace && \
    tar -xzf /tmp/uphf_gyrovia.tar.gz -C /workspace && \
    rm /tmp/uphf_gyrovia.tar.gz && \
    find /workspace -path "*uphf_gyrovia*TM*.bin" -delete

# Definir la piste UPHF comme map de demarrage par defaut
RUN sed -i \
    's#GameDefaultMap=/Game/Carla/Maps/Town10HD_Opt.Town10HD_Opt#GameDefaultMap=/Game/uphf_gyrovia/Maps/uphf_gyrovia/uphf_gyrovia.uphf_gyrovia#' \
    /workspace/CarlaUE4/Config/DefaultEngine.ini

# Python 3.10 portable (standalone, aucune dependance systeme) + client carla
COPY python310-standalone.tar.gz /opt/python310.tar.gz
RUN mkdir -p /opt/python310 && \
    tar -xzf /opt/python310.tar.gz -C /opt/python310 --strip-components=1 && \
    rm /opt/python310.tar.gz && \
    /opt/python310/bin/python3.10 -m pip install \
        /workspace/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-manylinux_2_31_x86_64.whl && \
    /opt/python310/bin/python3.10 -c "import carla; print('carla client OK')"

# Script de lancement
COPY launch_uphf.sh /workspace/launch_uphf.sh
RUN chmod +x /workspace/launch_uphf.sh

USER carla
EXPOSE 2000-2002
ENTRYPOINT ["/workspace/launch_uphf.sh"]
