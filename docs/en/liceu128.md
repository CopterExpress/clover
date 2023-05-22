# "QCS" - the network of Clover charging stations

[CopterHack-2023](copterhack2023.md), team **Lyceum 128**.

## Network realisation

Our charging stations use Python web server created with Django framework. On that server we storage information about charging stations:

- Position (GPS + ArUco marker).
- Possibility to drone landing.
- Drone info (If it's on it).

To connect to server we use API with special personal key for every drone and station. It can be regenerated if secured key became public.

If you want to test station without drone you can use API Debug page. You must be in your account to open it.

### Electronics in the station

There are Arduino Mega and Wemos D1 on the station.

![scheme](https://github.com/Juli-Shvetsova/clover/assets/78372613/3ab05a79-0046-463b-83dd-4db06115909b)

Wemos D1 connect with server to collect information, do tasks. Arduino Mega receive signals from Wemos and make physical updates such as moving landing platform, LED indication and other more.

After completing mission Wemos send request to a server to confirm updates on the server.

## Clover flight

We're using recursive landing algorithm to achieve success landing. Small ArUco marker is on the landing platform. Camera can use this marker on the ~25cm height. Next drone use standard landing.

## Visit our landing and API page

[https://qcs.pythonanywhere.com/](https://qcs.pythonanywhere.com/)

## Source code

Of that project is in our [GitHub page](https://github.com/qcs-charge/).

## Team

CH2023, Lyceum 128.

- Mikhail Konstantinov, [@mikemka](https://t.me/mikemka/), programmer.
- Julia Shvecova, [@Juli_Phil](https://t.me/Juli_Phil/), science adviser.
- Oleg Sherstobitov, [@kulumuluu](https://t.me/kulumuluu/), constructor.
