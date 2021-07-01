# Docker for beginners 

In this article we will understand a basic overview of docker, docker image, and docker containers. It will also provides the step by step instructions for;

1. Create docker image, build and run it in a container.
2. Push the docker image to docker hub.

## What is Docker 

Docker is a framework to build, run, and ship applications in a way that it run in a consistent way on the other machines. 

``` cpp
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

int main()
{
    Eigen::Matrix3d Rotation; 
    Rotation.setIdentity();
    std::cout<<Rotation<<std::endl;
    return 0;
}
```

``` cmake
cmake_minimum_required(VERSION 2.8)
project(docker_example) 
add_compile_options(-std=c++11) 
find_package(Eigen3 REQUIRED)
link_directories(${Eigen_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} "src/main.cpp") 
target_link_libraries(${PROJECT_NAME}  ${Eigen_LIBRARIES} )
```

``` docker
FROM ubuntu:bionic

RUN apt-get -y update &&    \
    apt-get install -y      \
    libeigen3-dev           \
    cmake

COPY . /application

WORKDIR /application

RUN cd /application;        \
    mkdir build;            \
    pwd && ls -la;          \
    cd build;               \
    cmake ..;               \
    pwd && ls -la;          \
    make; 

WORKDIR /application/build

CMD "./docker_example"


```
