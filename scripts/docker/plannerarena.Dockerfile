FROM rocker/shiny-verse:4.0.3
RUN apt-get update && \
    apt-get install -y libv8-dev libjpeg-dev
RUN install2.r --error --deps TRUE \
    shinyjs \
    V8 \
    pool \
    Hmisc \
    RSQLite \
    markdown && \
    rm -rf /tmp/downloaded_packages/
COPY plannerarena /srv/shiny-server/plannerarena
COPY docker/plannerarena.conf /etc/shiny-server/shiny-server.conf
ADD --chown=shiny:shiny https://www.cs.rice.edu/~mmoll/default-benchmark.db \
    /srv/shiny-server/plannerarena/www/benchmark.db
