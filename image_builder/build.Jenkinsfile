pipeline {
  agent any
  parameters {
    string(name: 'IMAGE_NAME', defaultValue: 'clever_noname.img', description: 'Output image file name')
    string(name: 'GWBT_REF', defaultValue: 'master', description: 'Checkout ref-param')
    string(name: 'IMAGE_VERSION', defaultValue: 'no_version', description: 'Image version')

    string(name: 'BUILD_DIR', defaultValue: '/mnt/hdd_builder/workspace', description: 'Build workspace')
    string(name: 'MOUNT_POINT', defaultValue: '/mnt/hdd_builder/image', description: 'Mount point')

    string(name: 'RPI_DONWLOAD_URL', defaultValue: 'https://downloads.raspberrypi.org/raspbian_lite/images/raspbian_lite-2017-12-01/2017-11-29-raspbian-stretch-lite.zip')
    // TODO: Add mirrorparameters

    string(name: 'GWBT_URL', defaultValue: 'https://github.com/CopterExpress/clever.git')
  }
  environment {
    DEBIAN_FRONTEND = 'noninteractive'
    LANG = 'C.UTF-8'
    LC_ALL = 'C.UTF-8'
  }
  stages {
    stage('Get image') {
      steps {
        sh "$WORKSPACE/image_builder/image_config.sh get_image ${params.BUILD_DIR} ${params.RPI_DONWLOAD_URL} ${params.IMAGE_NAME}"
      }
    }
    stage('Resize FS') {
      environment {
        SIZE = '7G'
      }
      steps {
        sh "$WORKSPACE/image_builder/image_config.sh resize_fs $SIZE ${params.BUILD_DIR} ${params.IMAGE_NAME}"
      }
    }
    stage('Initialize image') {
      environment {
        EXECUTE_FILE = 'image_builder/scripts/init_image.sh'
      }
      // TODO: Transfer apps.sh initialisation code here
      steps {
        sh "$WORKSPACE/image_builder/image_config.sh execute ${params.BUILD_DIR}/${params.IMAGE_NAME} ${params.MOUNT_POINT} $WORKSPACE/$EXECUTE_FILE ${params.IMAGE_VERSION} \$(basename ${params.RPI_DONWLOAD_URL})"
      }
    }
    stage('Hardware setup') {
      environment {
        EXECUTE_FILE = 'image_builder/scripts/hardware_setup.sh'
      }
      steps {
        sh "$WORKSPACE/image_builder/image_config.sh execute ${params.BUILD_DIR}/${params.IMAGE_NAME} ${params.MOUNT_POINT} $WORKSPACE/$EXECUTE_FILE"
      }
    }
    stage('Software install') {
      environment {
        EXECUTE_FILE = 'image_builder/scripts/software_install.sh'
      }
      steps {
        sh "$WORKSPACE/image_builder/image_config.sh execute ${params.BUILD_DIR}/${params.IMAGE_NAME} ${params.MOUNT_POINT} $WORKSPACE/$EXECUTE_FILE"
      }
    }
    stage('Network setup') {
      environment {
        EXECUTE_FILE = 'image_builder/scripts/network_setup.sh'
      }
      steps {
        sh "$WORKSPACE/image_builder/image_config.sh execute ${params.BUILD_DIR}/${params.IMAGE_NAME} ${params.MOUNT_POINT} $WORKSPACE/$EXECUTE_FILE"
      }
    }
    stage('Install ROS') {
      environment {
        EXECUTE_FILE = 'image_builder/scripts/ros_install.sh'
      }
      steps {
        sh "$WORKSPACE/image_builder/image_config.sh execute ${params.BUILD_DIR}/${params.IMAGE_NAME} ${params.MOUNT_POINT} $WORKSPACE/$EXECUTE_FILE"
      }
    }
    // TODO: Add finalising step, transfer mirror removal from ros.sh
  }
}
