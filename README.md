# Urpobotti

Robot for HSF15 compo

## TODO

  - Lots of things but streaming Pixy image data would be cool http://www.cmucam.org/boards/9/topics/5106?r=5162

## ZMQ ports

  - 7574: IMU data publisher 
  - 7575: Motorcontroller RPC
  - 7576: Motorcontroller data publisher
  - 7577: Pinger RPC
  - 7578: Pinger data publisher
  - 7579: Pixy RPC
  - 7580: Pixy data publisher

# Compiling/installing things

## Python stuff

<pypi.python.org/pypi/zmqdecorators> install this and dependencies.

TODO: Check the other deps and list them here

## Pixy stuff

Start by compiling and installing [libpixyusb](http://cmucam.org/projects/cmucam5/wiki/Building_the_libpixyusb_example_on_Linux). TLDR:

    git clone https://github.com/charmedlabs/pixy.git
    cd pixy/scripts/
    sudo apt-get install libusb-1.0-0-dev g++ libboost1.48-all-dev cmake 
    ./build_libpixyusb.sh
    sudo ./install_libpixyusb.sh
    ./build_hello_pixy.sh

Then you can compile the ZMQ pixy server

    sudo apt-get install libzmq-dev
    cd cpp/pixyzmq
    cmake ./
    make

