#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/natalia/ros_baxter/src/baxter_tools"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/natalia/ros_baxter/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/natalia/ros_baxter/install/lib/python2.7/dist-packages:/home/natalia/ros_baxter/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/natalia/ros_baxter/build" \
    "/usr/bin/python" \
    "/home/natalia/ros_baxter/src/baxter_tools/setup.py" \
    build --build-base "/home/natalia/ros_baxter/build/baxter_tools" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/natalia/ros_baxter/install" --install-scripts="/home/natalia/ros_baxter/install/bin"
