## Setup your builder

1. Install requirements
    ```bash
    sudo apt-get install unzip zip git python-pip jq curl
    sudo pip install YaDiskClient
    ```
2. Mount HDD
    ```bash
    nano /etc/fstab
    ```
    ```
    proc            /proc           proc    defaults          0       0
    PARTUUID=37665771-01  /boot           vfat    defaults          0       2
    PARTUUID=37665771-02  /               ext4    defaults,noatime  0       1
    # a swapfile is not a swap partition, no line here
    #   use  dphys-swapfile swap[on|off]  for that
    /dev/sdb1       none                    swap    sw              0       0
    /dev/sdb2       /mnt/hdd_system         ext4    defaults,acl    0       0
    /dev/sdb3       /mnt/hdd_builder        ext4    defaults,acl    0       0
    ```

3. Enable swap on HDD
    > TODO
4. And disable `dphys-swapfile`
    ```bash
    sudo systemctl stop dphys-swapfile
    sudo systemctl disable dphys-swapfile
    ```
5. Create /mnt/hdd_builder/workspace/coex-ci.json
    ```(json)
    {
        "yadisk":
        {
            "login":"LOGIN",
            "password":"PASS",
            "server_dir":"/clever_images"
        },
        "github":
        {
            "login":"LOGIN",
            "password":"PASS",
            "url":"https://api.github.com/repos/CopterExpress/clever/releases/"
        }
    }
    ```
6. Add webhook to release on your github project
    > TODO
7. Install Jenkins
    > Manual https://www.digitalocean.com/community/tutorials/how-to-install-jenkins-on-ubuntu-16-04
8. Change user & group invoked Jenkins
    ```bash
    sudo sed -i 's/JENKINS_USER=$NAME/JENKINS_USER=root/' /etc/default/jenkins
    sudo sed -i 's/JENKINS_GROUP=$NAME/JENKINS_GROUP=root/' /etc/default/jenkins
    ```
9. Install Jenikins plugins
    > Pipeline, Git SCM, Matrix Authorization, github-webhook-build-trigger-plugin
10. Create Jenkins pipeline job
    > TODO
11. Configure Jenkins
    > TODO: Matrix autorization, GIT Token
12. Add Jenkins service to autostart
    ```bash
    sudo systemctl enable jenkins
    ```
13. Start service
    ```bash
    sudo systemctl start jenkins
    ```

## Requirements

* Jenkins (BlueOcean plugin, optional)

## Troubleshooting

If JDK not installed:

```bash
sudo apt-get install default-jdk
```

## Для использования execute в качестве mount_image

```bash
./image_config.sh execute $IMAGE_PATH << EOF
uname -a
EOF
```

## Running the Docker

```bash
docker run --privileged -it --rm -v /dev:/dev -v $(pwd)/image:/builder/image smirart/builder
```

## TODO

* Change http на https в jenkins plugins
* Add finally block for disconnect image
* In Jenkins build call by name - change

**image-chroot**

* Добавить отмонтирование образа при возникновении ошибке
* проверка на существование образа и скрипта если тот задан (кстати скрипт копируется перед исполнением, модет подумать как сопрягать с copytochroot)

**image-build**

* должна смотреть в текущую папку
* проверка на существование файла инструкций
* Тоесть по сути должен быть отдельный скрипт для скачивания репы, если таковой не имеется
* Идея для билдера: добавить в Volume репу с инструкциями. И делать лишь git fetch, git pull, git checkout
* Сделать так, чтобы в текущей папке все собиралось и работало:
    1. Если это репозиторий (как передавать доступ в папку?)
    2. Брать имя репы, ветку или бренчу и коммит из файлов репы
    3. Обеспечить возможность работы с удаленным репозиторием в качестве источника инструкций (может сделать переход по коммиту или еще что-то подобное)

**image-resize <IMAGE> free-space**

## Варнинги Jenkins

```log
[WARNING] The POM for org.jenkins-ci.tools:maven-hpi-plugin:jar:2.0 is missing, no dependency information available
[WARNING] Failed to build parent project for io.codeclou.jenkins.github.webhook.build.trigger.plugin:github-webhook-build-trigger-plugin:hpi:1.2.0
```

* https://yandex.ru/search/?text=The%20POM%20for%20org.jenkins-ci.tools%3Amaven-hpi-plugin%3Ajar%3A2.0%20is%20missing%2C%20no%20dependency%20information%20available&&lr=213
* http://jenkins-ci.361315.n4.nabble.com/Plugin-org-jenkins-ci-tools-maven-hpi-plugin-td4751140.html
* http://qaru.site/questions/1460710/maven-jenkins-plugin-poms-missing-for-dependency-information-on-jars

