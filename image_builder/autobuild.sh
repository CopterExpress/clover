#! /usr/bin/env bash

set -e

export DEBIAN_FRONTEND=${DEBIAN_FRONTEND:='noninteractive'}
export LANG=${LANG:='C.UTF-8'}
export LC_ALL=${LC_ALL:='C.UTF-8'}

export TARGET_REPO=${TARGET_REPO:='https://github.com/urpylka/clever.git'}
export TARGET_REF=${TARGET_REF:='qemu_test_2'}
export TARGET_CONFIG=${TARGET_CONFIG:='/image_builder/scripts/builder.json'}

echo "================================================================================"
echo "TARGET_REPO: $TARGET_REPO"
echo "TARGET_REF: $TARGET_REF"
echo "TARGET_CONFIG: $TARGET_CONFIG"
echo "================================================================================"

# TODO: The repository can be already downloaded, use the TARGET_REPO also as unix path.
REPO_DIR=$(mktemp -d --suffix=.builder_repo)
git clone ${TARGET_REPO} --single-branch --branch ${TARGET_REF} --depth 1 ${REPO_DIR} \
|| (echo 'Error: Could not clone repo!'; return 1)
[[ -f ${REPO_DIR}${TARGET_CONFIG} ]] && export TARGET_CONFIG=${REPO_DIR}${TARGET_CONFIG} \
|| (echo "Error: TARGET_CONFIG doesn't exist!"; return 1)

export IMAGE_VERSION="${TARGET_REF}_$(date '+%Y%m%d_%H%M%S')"
export IMAGE_PATH="$(pwd)/image/$(basename -s ".git" ${TARGET_REPO})_${IMAGE_VERSION}.img"

./image_config.sh get_image ${IMAGE_PATH} $(jq '.source_image' -r ${TARGET_CONFIG})

if [[ $(arch) != 'armv7l' ]]; then
  [[ -d '/proc/sys/fs/binfmt_misc' ]] || mount binfmt_misc -t binfmt_misc /proc/sys/fs/binfmt_misc
  mount binfmt_misc -t binfmt_misc /proc/sys/fs/binfmt_misc
  echo ':arm:M::\x7fELF\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x28\x00:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/bin/qemu-arm-static:' > /proc/sys/fs/binfmt_misc/register 2&> /dev/null
  ./image_config.sh copy_to_chroot ${IMAGE_PATH} './qemu-arm-resin' '/usr/bin/qemu-arm-static'
fi

export IMAGE_BUILDER="$(dirname $(readlink -e "$0"))"
export SCRIPTS_DIR="$(jq '.scripts_dir' -r ${TARGET_CONFIG})"

${REPO_DIR}$(jq '.main_script' -r ${TARGET_CONFIG})
