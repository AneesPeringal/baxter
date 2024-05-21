#! /bin/bash

current_version="5.1.0"
previous_version="5.0.4"
pkg_list=("intera_interface" "intera_core_msgs")
mkdir -p ./$current_version
cp ./$previous_version/index.html ./$current_version
# TODO: potentially use a template for this:
# Right now we need to update index.html with
#                - Archived versions
#                - Timestamps for generation
# update the top level index.html for current_version
for pkg in ${pkg_list[*]}
do
    pkg_path=$(rospack find $pkg)
    echo ">>> Documenting $pkg for release $current_version"
    mkdir -p ./$current_version/$pkg
    echo ">>> Temporarily switching directories to $pkg package.."
    pushd $pkg_path
    git fetch origin
    current_branch=$(git rev-parse --abbrev-ref HEAD)
    # unique timestamp
    t=timestamp-$(date +%s)
    # stash with message
    r=$(git stash save $t)
    # check if the value exists
    stash_success=$(echo $r|grep $t)
    echo ">>> Checking out branch 'tmp_docs' from origin/release-$current_version"
    git checkout -b tmp_docs origin/release-$current_version
    echo ">>> Generating ROS documentation for $pkg"
    rosdoc_lite $pkg_path
    echo ">>> Restoring repo to its original state"
    git checkout $current_branch
    git branch -d tmp_docs
    if [ "$stash_success" ]; then
        echo ">>> Reverting $pkg git directory state"
        git stash pop
    fi
    echo ">>> Changing directory back to intera_sdk_docs"
    popd
    echo ">>> Moving generated docs contents for $pkg into $current_version folder"
    rm -r ./$current_version/$pkg/*
    mv $pkg_path/doc/* ./$current_version/$pkg
    rm -r $pkg_path/doc
done

