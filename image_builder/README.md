## Setup your builder

1. Install requirements
```(bash)
sudo apt-get install unzip zip git python-pip jq curl
sudo pip install YaDiskClient
```
2. Mount HDD
> TODO

3. Enable swap on HDD
> TODO:

And disable `dphys-swapfile`
```(bash)
sudo systemctl stop dphys-swapfile
sudo systemctl disable dphys-swapfile
```

3. Create /mnt/hdd_builder/workspace/coex-ci.json
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

```
6. Add webhook to release on your github project
> TODO

9. Install Jenkins
> Manual https://www.digitalocean.com/community/tutorials/how-to-install-jenkins-on-ubuntu-16-04

10. Change user & group invoked Jenkins
```(bash)
sudo sed -i 's/JENKINS_USER=$NAME/JENKINS_USER=root/' /etc/default/jenkins
sudo sed -i 's/JENKINS_GROUP=$NAME/JENKINS_GROUP=root/' /etc/default/jenkins
```
11. Install Jenikins plugins
> Pipeline, Git SCM

12. Create Jenkins pipeline job
> TODO

13. Configure Jenkins
> TODO: Matrix autorization, GIT Token

13. Add Jenkins service to autostart
```(bash)
sudo systemctl enable jenkins
```

14. Start service
```(bash)
sudo systemctl start jenkins
```

## Requirements

* Jenkins (BlueOcean plugin, optional)
