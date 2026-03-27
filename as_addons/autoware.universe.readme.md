autoware.universe image
-----------------------

This image contains sources and precompiled binaries of autoware.universe repository
built as a part of AS_Drive project development.

Built from: %%BUILD_REF%%

Content:

- /opt/autoware.universe

    ROS overlay with precompiled version of autoware.universe repository. Can be used
    by sourcing /opt/autoware.universe/setup.bash file.

- /opt/autoware.universe/ros-deps

    Script for installing dependencies required by autoware.universe repository.
    Can be used to install required dependencies in a new image after
    /opt/autoware.universe folder is copied to it.

- /work/autoware_universe_ws

    ROS workspace with Autowaautoware.universereAuto sources used to built
    /opt/autoware.universe overlay.

- /work/autoware.universe.build_depends.repos

    File with list of additional repositories required to built autoware.universe
    repository. Those repositories should be already present inside
    /work/autoware_universe_ws/src folder.
