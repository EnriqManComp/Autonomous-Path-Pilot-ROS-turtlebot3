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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/media/enrique/ae414e9e-3895-46c8-92d6-d4cb9803e8e8/enrique/Road-Lane-Detection-ROS-turtlebot3/src/turtlebot3/turtlebot3_teleop"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/media/enrique/ae414e9e-3895-46c8-92d6-d4cb9803e8e8/enrique/Road-Lane-Detection-ROS-turtlebot3/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/media/enrique/ae414e9e-3895-46c8-92d6-d4cb9803e8e8/enrique/Road-Lane-Detection-ROS-turtlebot3/install/lib/python3/dist-packages:/media/enrique/ae414e9e-3895-46c8-92d6-d4cb9803e8e8/enrique/Road-Lane-Detection-ROS-turtlebot3/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/media/enrique/ae414e9e-3895-46c8-92d6-d4cb9803e8e8/enrique/Road-Lane-Detection-ROS-turtlebot3/build" \
    "/usr/bin/python3" \
    "/media/enrique/ae414e9e-3895-46c8-92d6-d4cb9803e8e8/enrique/Road-Lane-Detection-ROS-turtlebot3/src/turtlebot3/turtlebot3_teleop/setup.py" \
     \
    build --build-base "/media/enrique/ae414e9e-3895-46c8-92d6-d4cb9803e8e8/enrique/Road-Lane-Detection-ROS-turtlebot3/build/turtlebot3/turtlebot3_teleop" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/media/enrique/ae414e9e-3895-46c8-92d6-d4cb9803e8e8/enrique/Road-Lane-Detection-ROS-turtlebot3/install" --install-scripts="/media/enrique/ae414e9e-3895-46c8-92d6-d4cb9803e8e8/enrique/Road-Lane-Detection-ROS-turtlebot3/install/bin"
