FROM rocker/shiny-verse
RUN apt-get update && \
    apt-get install -y libv8-dev
RUN install2.r --error --deps TRUE \
    shinyjs \
    V8 \
    Hmisc \
    RSQLite \
    markdown
COPY plannerarena /srv/shiny-server/plannerarena
COPY docker/plannerarena.conf /etc/shiny-server/shiny-server.conf
ADD --chown=shiny:shiny https://mmoll.rice.edu/default-benchmark.db \
    /srv/shiny-server/plannerarena/www/benchmark.db
