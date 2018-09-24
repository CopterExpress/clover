#! /usr/bin/env bash

#
# Script for upload the image to yadisk & change the release message on GitHub
# Copyright (C) 2018 Copter Express Technologies
#
# Author: Artem Smirnov <urpylka@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -e # Exit immidiately on non-zero result

publish_image() {
  # TEMPLATE: publish_image_bash <IMAGE_PATH> <YA_SCRIPT> <CONFIG_FILE> <RELEASE_ID> <RELEASE_BODY>

  # https://developer.github.com/v3/repos/releases/

  IMAGE_NAME=$(basename $1)
  BUILD_DIR=$(dirname $1)

  echo -e "Zip image"
  if [ ! -e "${BUILD_DIR}/${IMAGE_NAME}.zip" ]; then
    cd ${BUILD_DIR} && zip ${IMAGE_NAME}.zip ${IMAGE_NAME}
    echo -e "Zipping complete!" "SUCCESS"
  else
    echo -e "Zip-archive already created"
    cd ${BUILD_DIR} && rm ${IMAGE_NAME}.zip && zip ${IMAGE_NAME}.zip ${IMAGE_NAME} \
    && echo -e "Old archive was deleted & create new" "SUCCESS"
  fi

  echo -e "Upload image"
  local IMAGE_LINK=$($2 $3 ${BUILD_DIR}/${IMAGE_NAME}.zip) \
  && echo -e "Upload copmlete!" "SUCCESS"

  echo -e "Meashure size of zip-image"
  local IMAGE_SIZE=$(du -sh ${BUILD_DIR}/${IMAGE_NAME}.zip | awk '{ print $1 }') \
  && echo -e "Meashuring copmlete!" "SUCCESS"

  echo -e "Meashure hash-sum of zip-image"
  local IMAGE_HASH=$(sha256sum ${BUILD_DIR}/${IMAGE_NAME}.zip | awk '{ print $1 }') \
  && echo -e "Meashuring copmlete!" "SUCCESS"

  echo ""
  echo "\$5: $5"
  echo ""

  echo -e "Post message to GH"
  local NEW_RELEASE_BODY="### Download\n* [${IMAGE_NAME}.zip](${IMAGE_LINK}) (${IMAGE_SIZE})\nsha256: ${IMAGE_HASH}\n\n$5"
  local DATA="{ \"body\":\"${NEW_RELEASE_BODY}\" }"

  echo ""
  echo "\$DATA: $DATA"
  echo ""

  local GH_LOGIN=$(cat $3 | jq '.github.login' -r)
  local GH_PASS=$(cat $3 | jq '.github.password' -r)
  local GH_URL=$(cat $3 | jq '.github.url' -r)
  curl -d "$DATA" -u "${GH_LOGIN}:${GH_PASS}" --request PATCH ${GH_URL}$4 \
  && echo -e "Post message to GH copmlete!" "SUCCESS"
}

publish_image $1 $2 $3 $4 "$5"
