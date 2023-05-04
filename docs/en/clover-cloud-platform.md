# Clover Cloud Platform

[CopterHack-2023](copterhack2023.md), team **Clover Cloud Team**.

The list of our team members:

* Кирилл Лещинский / Kirill Leshchinskiy, [@k_leshchinskiy](https://t.me/k_leshchinskiy) - Team Lead.
* Кузнецов Михаил / Mikhail Kuznetsov, [@bruhfloppa](https://t.me/bruhfloppa) - Frontend Developer.
* Даниил Валишин / Daniil Valishin, [@Astel_1](https://t.me/Astel_1) - Backend Developer.

## Table of contents

* [Introduction](#introduction)
* [Usability](#usability)
* [How to work with our platform?](#how-to-work-with-our-platform)
* [About the development of the platform](#about-the-development-of-the-platform)
* [Conclusion](#conclusion)

## Video demonstration

<p align="center">
  <a href="https://www.youtube.com/watch?v=FZPl2LOMgi4"><img img width="560" height="315" src="https://img.youtube.com/vi/FZPl2LOMgi4/maxresdefault.jpg" /></a>
</p>

## Introduction

Clover Cloud Platform is an innovative platform that enables users to access COEX Clover drone simulation online, without the need to download any programs or virtual machines.

>**Visit our [documentation](https://docs.clovercloud.software) to learn all about the platform, its development and how to use it.**

## Unleash Your Coding Power: Develop Autonomous Flight Code at Lightning Speed on Clover Cloud Platform

If you're a developer working on autonomous flight projects, you know how time-consuming and distracting all of the routine activities can be. Between managing your hardware, debugging, and configuring your environment, it can feel like the real work of coding gets lost in the shuffle.

That's where our platform comes in. Our streamlined interface and powerful tools make it easy to tackle all of those essential tasks so you can focus on what really matters: developing flawless, high-performance code for your autonomous flight project.

So why wait to unleash your coding power? Sign up for our platform today and discover the difference it can make in the speed, quality, and focus of your autonomous flight coding work.

## Usability

Our platform is incredibly user-friendly and provides seamless access to the simulation in just a few clicks. Together with a simulator that displays simulation data accurately and without delay, there is a map editor allows users to edit the ArUco marker map and add or modify other objects on the scene directly within the simulation window. Additionally, users can create pre-configured workspaces complete with autonomous flight code and simulation scene configuration. Each user can also create their templates or apply a pre-made one to their workspace in just a few clicks. In addition to its other features, Clover Cloud Platform provides users with a convenient code editor for autonomous flight coding. Users can write code in the built-in editor and run it directly from the editor, viewing program output in real-time in the terminal. The platform also includes a file manager that simplifies file manipulation tasks, further enhancing the user's overall experience. With these tools at your fingertips, Clover Cloud Platform delivers an unparalleled level of accessibility and convenience for autonomous flight simulation.

<p align="center">

![Workspace screenshot](https://raw.githubusercontent.com/Clover-Cloud-Platform/clover-cloud-platform-frontend/master/docs/workspace.png)

</p>

## The CodeSandbox for COEX Clover

You can describe the usability and relevance of our platform in another way. Have you heard of CodeSandbox? Our platform offers the same convenience, flexibility, and accessibility as CodeSandbox, but is specifically designed to work with the COEX Clover drone simulation.

## How to work with our platform?

Let's dive into the sea of functionality that our platform offers. Detailed description of each feature is available in our [documentation](https://docs.clovercloud.software), here we will provide a general overview of the platform.

### Creating an account

First, you should create an account on our site. You can do this by clicking on this [link](https://clovercloud.software/signup).

### Instance management

After creating an account, you will be taken to the [dashboard](https://clovercloud.software/instances). Here you can create, start, stop and delete workspaces.

>Workspaces are containers with Gazebo simulator and our software that provide data flow for simulation visualization, as well as handle requests from file manager, code editor and terminal.

<p align="center">

![instance management](https://raw.githubusercontent.com/Clover-Cloud-Platform/clover-cloud-platform-frontend/master/docs/instances.gif)

</p>

### Workspace overview

In the workspace, in addition to the simulator, you have a file manager, code editor and terminal. There is also an editing mode in the simulator - one of the key features of our project. It allows you to quickly and conveniently edit the simulation scene, namely: move ArUco markers, change their size, change id of the marker, load instead of marker picture, add new markers or delete them. You can also add 3d objects to the scene and change their position, size and color. Below is an example of working with our workspace.

<p align="center">

![workspace overview](https://github.com/Clover-Cloud-Platform/clover-cloud-platform-frontend/raw/master/docs/workspace.gif)

</p>

### Templates

Templates are another key feature of our platform.Is there something you can't do and you want to see how to properly perform a task? Look for the right template with ready-made code in the Template Browser and apply it to your workspace! Each user can create a template with an autonomous flight code and simulator configuration and share it.

## About the development of the platform

Our team has worked tirelessly to develop a simple yet multifunctional platform. We utilized the most modern standards and tools and implemented numerous optimization methods to ensure seamless performance and error-free operation. The frontend programming language chosen was JavaScript with the React framework, as a design system we utilizing Material Design style for an elegant and intuitive user interface. With the help of GitHub Actions the website is being built and deployed to Firebase hosting. The platform's backend is written in Python and contains multiple simultaneously running scripts. User data is secured and stored in a MongoDB database. Communication between the server and site is enabled through web sockets and the socket.io library, guaranteeing lightning-fast data transfer with minimal lag.
>You can view the source code of our platform by clicking on the links below.<br/>
[Repository with the frontend-side code](https://github.com/Clover-Cloud-Platform/clover-cloud-platform-frontend)<br/>
[Repository with the backend-side code](https://github.com/Clover-Cloud-Platform/clover-cloud-platform-backend)

## Conclusion

In conclusion, we have successfully created a truly convenient and useful platform, suitable for both novice and advanced COEX Clover drone users. Beginners can test their first autonomous flight code without the need for demanding simulator installation or virtual machines. They can also explore all of the drone's functions and capabilities without editing any configuration files. Advanced users benefit from access to their workspace from anywhere in the world and on any device, along with a convenient code-sharing system. In the future, we plan to add more new features to our platform, scale our network to serve a greater number of users, and collaborate with COEX to integrate their Clover quadcopter documentation into our platform, offering users a very simple and user-friendly way to learn to program autonomous drone flight. We also want to express gratitude to the COEX customer support team for their assistance in resolving complex issues that arose during development.
