# Работа с камерой

Для работы с основной камерой необходимо убедиться что она включена в файле `~/catkin_ws/src/clever/clever/launch/clever.launch`:

```xml
<arg name="main_camera" default="true"/>
```

При изменении launch-файла необходимо перезапустить пакет `clever`:

```bash
sudo systemctl restart clever
```

Для мониторинга изображения с камеры возможно использовать rqt или [web_video_server](/docs/web_video_server.md).

## Компьютерное зрение

Для реализации алгоритмов компьютерного зрения рекомендуется использовать предустановленную на [образ SD-карты](/docs/microsd_images.md) библиотеку OpenCV.

### Python

Для обработки изображений с камеры в [OpenCV](https://opencv.org) в Python необходимо создать подписчика 