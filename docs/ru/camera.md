# Работа с камерой

<!-- TODO: физическое подключение -->

Для работы с основной камерой необходимо убедиться что она включена в файле `~/catkin_ws/src/clever/clever/launch/clever.launch`:

```xml
<arg name="main_camera" default="true"/>
```

Также нужно убедиться, что для камеры [указано корректное расположение и ориентация](camera_frame.md).

При изменении launch-файла необходимо перезапустить пакет `clever`:

```bash
sudo systemctl restart clever
```

Для мониторинга изображения с камеры можно использовать [rqt](rviz.md) или [web_video_server](web_video_server.md).

## Неисправности

Если изображение с камеры отсутствует, попробуйте проверить ее с помощью утилиты [`raspistill`](https://www.raspberrypi.org/documentation/usage/camera/raspicam/raspistill.md).

Остановите сервисы Клевера:

```bash
sudo systemctl stop clever
```

Получите картинку с камеры утилитой `raspistill`:

```bash
raspistill -o test-image.jpg
```

Если команда завершается с ошибкой, проверьте качество подключения шлейфа камеры к Raspberry Pi или замените его.

## Настройки камеры

Ряд параметров камеры - размер изображения, максимальную частоту кадров, экспозицию - можно настроить в файле `main_camera.launch`. Список настраиваемых параметров можно [посмотреть в репозитории cv_camera](https://github.com/OTL/cv_camera#parameters).

Параметры, не указанные в этом списке, можно указывать через [код параметра OpenCV](https://docs.opencv.org/3.3.1/d4/d15/group__videoio__flags__base.html). Например, для установки фиксированной экспозиции добавьте следующие параметры в ноду камеры:

```xml
<param name="property_0_code" value="21"/> <!-- property code 21 is CAP_PROP_AUTO_EXPOSURE -->
<param name="property_0_value" value="0.25"/> <!-- property values are normalized as per OpenCV specs, even for "menu" controls; 0.25 means "use manual exposure" -->
<param name="cv_cap_prop_exposure" value="0.3"> <!-- set exposure to 30% of maximum value -->
```

## Компьютерное зрение

Для реализации алгоритмов компьютерного зрения рекомендуется использовать предустановленную на [образ SD-карты](image.md) библиотеку [OpenCV](https://opencv.org).

### Python

Основная статья: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython.

Пример создания подписчика на топик с изображением с основной камеры для обработки с использованием OpenCV:

```python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('computer_vision_sample')
bridge = CvBridge()

def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    # Do any image processing with cv2...

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

rospy.spin()
```

Для отладки обработки изображения можно публиковать отдельный топик с обработанным изображением:

```python
image_pub = rospy.Publisher('~debug', Image)
```

Публикация обработанного изображения (в конце функции image_callback):

```python
image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
```

Получаемые изображения можно просматривать используя [web_video_server](web_video_server.md).

> **Warning** По умолчанию web_video_server показывает изображения из топиков со сжатием (например, /main_camera/image_raw/compressed). Ноды на Python не публикуют такие топики, поэтому для их просмотра следует добавлять `&type=mjpeg` в адресную стоку страницы web_video_server или изменить параметр `default_stream_type` на `mjpeg` в файле `clever.launch`.

### Примеры

#### Работа с QR-кодами

> **Hint** Для высокоскоростного распознавания и позиционирования лучше использовать [ArUco-маркеры](aruco.md).

Для программирования различных действий коптера при детектировании нужных [QR-кодов](https://ru.wikipedia.org/wiki/QR-код) можно использовать библиотеку [ZBar](http://zbar.sourceforge.net). Ее нужно установить в помощью pip:

```bash
sudo pip install zbar
```

Распознавание QR-кодов на Python:

```python
import cv2
import zbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()
scanner = zbar.ImageScanner()
scanner.parse_config('enable')

# Image subscriber callback function
def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY, dstCn=0)

    pil = ImageZ.fromarray(gray)
    raw = pil.tobytes()

    image = zbar.Image(320, 240, 'Y800', raw)  # Image params
    scanner.scan(image)

    for symbol in image:
        # print detected QR code
        print 'decoded', symbol.type, 'symbol', '"%s"' % symbol.data

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)
```

Скрипт будет занимать 100% процессора. Для искусственного замедления работы скрипта можно запустить [throttling](http://wiki.ros.org/topic_tools/throttle) кадров с камеры, например, в 5 Гц (`main_camera.launch`):

```xml
<node pkg="topic_tools" name="cam_throttle" type="throttle"
    args="messages main_camera/image_raw 5.0 main_camera/image_raw/throttled"/>
```

Топик для подписчика в этом случае необходимо поменять на `main_camera/image_raw/throttled`.
