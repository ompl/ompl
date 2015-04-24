#!/bin/sh

BRANCH=trunk

# web server
export SERVER=arachne.cs.rice.edu
# directory on web server where different versions of the web site
# and other assets are located.
export ASSETS_ROOT=/mnt/data2/ompl/$BRANCH
export ASSET_DIR=ompl

echo "Exporting OMPL documentation for $BRANCH"

# copy everything to web server and fix permissions
tar cf - -s/html/${ASSET_DIR}/ html | ssh ${SERVER} 'mkdir -p '${ASSETS_ROOT}'; cd '${ASSETS_ROOT}'; tar xf -; chmod -R a+rX '${ASSET_DIR}'; chgrp -R ompl '${ASSET_DIR}'; chmod -R g+w '${ASSET_DIR}

# clean up
rm -rf ${ASSET_DIR}

echo The web site has been copied to:
echo "      " ${SERVER}:${ASSETS_ROOT}/${ASSET_DIR}.
