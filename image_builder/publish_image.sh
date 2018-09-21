#! /usr/bin/env bash

#
# Script for image configure
# @urpylka Artem Smirnov
# @dvornikov-aa Andrey Dvornikov
#

# Exit immidiately on non-zero result
set -e

source echo_stamp.sh

publish_image() {
  # TEMPLATE: publish_image_bash <IMAGE_PATH> <YA_SCRIPT> <CONFIG_FILE> <RELEASE_ID> <RELEASE_BODY>

  # https://developer.github.com/v3/repos/releases/

  IMAGE_NAME=$(basename $1)
  BUILD_DIR=$(dirname $1)

  echo_stamp "Zip image"
  if [ ! -e "${BUILD_DIR}/${IMAGE_NAME}.zip" ];
  then
    cd ${BUILD_DIR} && zip ${IMAGE_NAME}.zip ${IMAGE_NAME}
    echo_stamp "Zipping complete!" "SUCCESS"
  else
    echo_stamp "Zip-archive already created"
    cd ${BUILD_DIR} && rm ${IMAGE_NAME}.zip && zip ${IMAGE_NAME}.zip ${IMAGE_NAME} \
    && echo_stamp "Old archive was deleted & create new" "SUCCESS"
  fi

  echo_stamp "Upload image"
  local IMAGE_LINK=$($2 $3 ${BUILD_DIR}/${IMAGE_NAME}.zip) \
  && echo_stamp "Upload copmlete!" "SUCCESS"

  echo_stamp "Meashure size of zip-image"
  local IMAGE_SIZE=$(du -sh ${BUILD_DIR}/${IMAGE_NAME}.zip | awk '{ print $1 }') \
  && echo_stamp "Meashuring copmlete!" "SUCCESS"

  echo_stamp "Meashure hash-sum of zip-image"
  local IMAGE_HASH=$(sha256sum ${BUILD_DIR}/${IMAGE_NAME}.zip | awk '{ print $1 }') \
  && echo_stamp "Meashuring copmlete!" "SUCCESS"

  echo ""
  echo "\$5: $5"
  echo ""

  echo_stamp "Post message to GH"
  local NEW_RELEASE_BODY="### Download\n* [${IMAGE_NAME}.zip](${IMAGE_LINK}) (${IMAGE_SIZE})\nsha256: ${IMAGE_HASH}\n\n$5"
  local DATA="{ \"body\":\"${NEW_RELEASE_BODY}\" }"

  echo ""
  echo "\$DATA: $DATA"
  echo ""

  local GH_LOGIN=$(cat $3 | jq '.github.login' -r)
  local GH_PASS=$(cat $3 | jq '.github.password' -r)
  local GH_URL=$(cat $3 | jq '.github.url' -r)
  curl -d "$DATA" -u "${GH_LOGIN}:${GH_PASS}" --request PATCH ${GH_URL}$4 \
  && echo_stamp "Post message to GH copmlete!" "SUCCESS"
}

publish_image $1 $2 $3 $4 "$5"
