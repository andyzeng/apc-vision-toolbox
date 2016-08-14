g++ stream.cpp -std=c++11 -I/usr/local/include -Ithird_party -Iutil -lrealsense `pkg-config --cflags --libs glfw3 gl glu` `pkg-config --cflags opencv` `pkg-config --libs opencv` -o stream
