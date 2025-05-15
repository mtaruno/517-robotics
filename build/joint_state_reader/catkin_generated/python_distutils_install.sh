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

echo_and_run cd "/fetch_ws/src/cse481c/joint_state_reader"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/fetch_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/fetch_ws/install/lib/python3/dist-packages:/fetch_ws/build/joint_state_reader/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/fetch_ws/build/joint_state_reader" \
    "/usr/bin/python3" \
    "/fetch_ws/src/cse481c/joint_state_reader/setup.py" \
     \
    build --build-base "/fetch_ws/build/joint_state_reader" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/fetch_ws/install" --install-scripts="/fetch_ws/install/bin"
